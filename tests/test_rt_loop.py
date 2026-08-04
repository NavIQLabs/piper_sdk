import time

from piper_sdk.controller import RtLoop


def test_loop_achieves_nominal_rate():
    ticks = {"n": 0}
    loop = RtLoop(rate_hz=200.0, callback=lambda dt: ticks.__setitem__("n", ticks["n"] + 1),
                  name="rate_test")
    loop.start()
    time.sleep(0.4)
    loop.stop()
    # 200 Hz * 0.4s = ~80 ticks; allow scheduling slack
    assert ticks["n"] > 50


def test_loop_stops_and_no_ticks_after():
    ticks = {"n": 0}
    loop = RtLoop(rate_hz=100.0, callback=lambda dt: ticks.__setitem__("n", ticks["n"] + 1),
                  name="stop_test")
    loop.start()
    time.sleep(0.15)
    loop.stop()
    n_after = ticks["n"]
    time.sleep(0.15)
    assert ticks["n"] == n_after


def test_loop_watchdog_overrun():
    trips = []

    def cb(dt):
        time.sleep(0.1)  # way over max_step_s

    loop = RtLoop(rate_hz=50.0, callback=cb, name="trip_test",
                  max_step_s=0.02, on_timeout=lambda r: trips.append(r))
    loop.start()
    time.sleep(0.3)
    loop.stop()
    assert len(trips) >= 1
    assert loop.timeout_reason is not None


def test_loop_reports_achieved_hz():
    loop = RtLoop(rate_hz=100.0, callback=lambda dt: None, name="hz_test")
    loop.start()
    time.sleep(1.2)  # fill the 1s realtime-fps window
    hz = loop.achieved_hz
    loop.stop()
    assert hz > 0.0
    assert 50.0 < hz < 200.0


def test_loop_manual_trip():
    trips = []
    loop = RtLoop(rate_hz=100.0, callback=lambda dt: None, name="man_trip",
                  on_timeout=lambda r: trips.append(r))
    loop.start()
    time.sleep(0.05)
    loop.trip("manual")
    time.sleep(0.05)
    loop.stop()
    assert trips == ["manual"]
