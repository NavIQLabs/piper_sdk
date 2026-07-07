#!/bin/bash
# test_all.sh — Build and run all cpiper tests
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"

echo "=== cpiper Test Suite ==="
echo ""

# Setup vcan
echo "--- Setting up vcan0 ---"
bash "$SCRIPT_DIR/scripts/setup_vcan.sh"
echo ""

# Build
echo "--- Building cpiper ---"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)
echo ""

# Run protocol tests (no hardware needed)
echo "--- Running Protocol Unit Tests ---"
./test_protocol
echo ""

# Run CAN tests (requires vcan0)
echo "--- Running CAN Integration Tests ---"
sudo ./test_can
echo ""

# Run interface tests (requires vcan0)
echo "--- Running Interface Integration Tests ---"
sudo ./test_interface
echo ""

# Run Python comparison tests
echo "--- Running Python Comparison Tests ---"
cd "$PROJECT_DIR/.."
python3 "$SCRIPT_DIR/tests/test_compare.py"
echo ""

echo "=== All tests completed ==="
