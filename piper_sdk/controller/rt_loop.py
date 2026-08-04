#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Fixed-rate realtime loop for the controller subpackage.

A daemon thread runs ``callback(dt)`` at a nominal fixed rate using
``time.perf_counter`` scheduling (target clock, no busy-wait drift).

Safety features:

* **Jitter / deadline watchdog** - if a single callback call exceeds
  ``max_step_s`` or a loop tick is late by more than ``max_late_s``, the
  optional ``on_timeout`` handler fires (controllers wire this to emergency
  stop / motor disable).
* **Rate monitoring** - a ``C_FPSCounter`` instance tracks the achieved tick
  rate so the controller can verify it is keeping up.
'''

import threading
import time
from ..utils.fps import C_FPSCounter

class RtLoop:
    '''
    :param rate_hz: nominal loop frequency in Hz.
    :param callback: ``callable(dt)`` invoked each tick.
    :param name: unique name used by the FPS counter.
    :param max_step_s: hard deadline for a single callback (0 disables).
    :param max_late_s: allowed scheduling lateness before ``on_timeout`` (0 disables).
    :param on_timeout: ``callable(reason)`` fired on watchdog trips.
    '''
    def __init__(self, rate_hz=500.0, callback=None, name="ctrl",
                 max_step_s=0.0, max_late_s=0.0, on_timeout=None):
        if rate_hz <= 0:
            raise ValueError("rate_hz must be > 0")
        self._period = 1.0 / rate_hz
        self._callback = callback
        self._name = name
        self._max_step_s = max_step_s
        self._max_late_s = max_late_s
        self._on_timeout = on_timeout
        self._fps = C_FPSCounter(start_realtime_fps=True)
        self._fps.add_variable(name)
        self._fps.start()
        self._running = False
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self._last_tick = time.perf_counter()
        self._ticks = 0
        self._max_jitter = 0.0
        self._timeout_reason = None

    # --------------------------------------------------------------- info
    @property
    def rate_hz(self):
        return 1.0 / self._period

    @property
    def achieved_hz(self):
        return self._fps.get_real_time_fps(self._name, window=1.0)

    @property
    def ticks(self):
        return self._ticks

    @property
    def max_jitter(self):
        return self._max_jitter

    @property
    def timeout_reason(self):
        return self._timeout_reason

    def get_fps(self):
        return self._fps

    # ---------------------------------------------------------------- run
    def start(self):
        '''Start the loop thread (no-op if already running).'''
        with self._lock:
            if self._running:
                return
            self._running = True
            self._stop.clear()
            self._last_tick = time.perf_counter()
            self._ticks = 0
            self._max_jitter = 0.0
            self._timeout_reason = None
            self._thread = threading.Thread(target=self._run, daemon=True,
                                            name="RtLoop_%s" % self._name)
            self._fps.start()
            self._thread.start()

    def stop(self, timeout=1.0):
        '''Stop the loop thread and join it.'''
        with self._lock:
            if not self._running:
                return
            self._running = False
            self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
            self._thread = None
        self._fps.stop()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()
        return False

    def trip(self, reason):
        '''
        Manually trip the watchdog (e.g. from a controller safety check).
        Thread-safe; triggers ``on_timeout`` and stops the loop.
        '''
        if self._timeout_reason is not None:
            return
        self._trip(reason)

    # --------------------------------------------------------------- core
    def _run(self):
        next_t = time.perf_counter()
        while not self._stop.is_set():
            next_t += self._period
            now = time.perf_counter()
            lateness = now - next_t
            if lateness > 0.0:
                self._max_jitter = max(self._max_jitter, lateness)
                if self._max_late_s > 0.0 and lateness > self._max_late_s:
                    self._trip("tick late by %.4fs" % lateness)
                    return
            # sleep to the next deadline (absolute-clock scheduling)
            sleep = next_t - time.perf_counter()
            if sleep > 0.0:
                self._stop.wait(sleep)
                if self._stop.is_set():
                    break
            else:
                self._stop.wait(0.0)
            now = time.perf_counter()
            dt = now - self._last_tick
            self._last_tick = now
            self._ticks += 1
            self._fps.increment(self._name)
            if self._callback is not None:
                t0 = time.perf_counter()
                try:
                    self._callback(dt)
                except Exception as e:  # noqa: BLE001 - watchdog must always fire
                    self._trip("callback error: %s" % e)
                    return
                step = time.perf_counter() - t0
                if self._max_step_s > 0.0 and step > self._max_step_s:
                    self._trip("callback overrun %.4fs" % step)
                    return

    def _trip(self, reason):
        self._timeout_reason = reason
        self._running = False
        self._stop.set()
        if self._on_timeout is not None:
            try:
                self._on_timeout(reason)
            except Exception:  # noqa: BLE001
                pass
