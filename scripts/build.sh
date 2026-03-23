#!/usr/bin/env bash
# Build Mangle module for Move Anything (ARM64)
#
# Cross-compiles Bungee + DSP plugin and packages all files for release.
# Set CROSS_PREFIX to skip Docker (e.g., for native ARM builds).
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
MODULE_ID="mangle"
IMAGE_NAME="move-anything-mangle-builder"

# Check if we need Docker
if [ -z "$CROSS_PREFIX" ] && [ ! -f "/.dockerenv" ]; then
    echo "=== Mangle Module Build (via Docker) ==="
    echo ""

    # Build Docker image if needed
    if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
        echo "Building Docker image (first time only)..."
        docker build -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile" "$REPO_ROOT"
        echo ""
    fi

    # Run build inside container
    echo "Running build..."
    docker run --rm \
        -v "$REPO_ROOT:/build" \
        -u "$(id -u):$(id -g)" \
        -w /build \
        "$IMAGE_NAME" \
        ./scripts/build.sh

    echo ""
    echo "=== Done ==="
    exit 0
fi

# === Actual build (runs in Docker or with cross-compiler) ===
CROSS_PREFIX="${CROSS_PREFIX:-aarch64-linux-gnu-}"

cd "$REPO_ROOT"

echo "=== Building Mangle Module ==="
echo "Cross prefix: $CROSS_PREFIX"

# Create build directories
mkdir -p build/bungee
mkdir -p "dist/$MODULE_ID"

BUNGEE_DIR=src/dsp/bungee

# --- Step 1: Build pffft (static) ---
echo "Compiling pffft..."
${CROSS_PREFIX}gcc -O3 -fPIC -ffast-math -fno-finite-math-only -fno-exceptions \
    -c "$BUNGEE_DIR/submodules/pffft/pffft.c" -o build/bungee/pffft.o
${CROSS_PREFIX}gcc -O3 -fPIC -ffast-math -fno-finite-math-only -fno-exceptions \
    -c "$BUNGEE_DIR/submodules/pffft/fftpack.c" -o build/bungee/fftpack.o

# --- Step 2: Build Bungee sources (static, incremental) ---
echo "Compiling Bungee library..."
for src in $BUNGEE_DIR/src/*.cpp; do
    obj="build/bungee/$(basename "$src" .cpp).o"

    if [ ! -f "$obj" ] || [ "$src" -nt "$obj" ]; then
        echo "Compiling $(basename "$src")"
        ${CROSS_PREFIX}g++ -O3 -fPIC -std=c++20 -fwrapv \
            -I"$BUNGEE_DIR/submodules/eigen" \
            -I"$BUNGEE_DIR/submodules" \
            -I"$BUNGEE_DIR" \
            '-DBUNGEE_VISIBILITY=__attribute__((visibility("default")))' \
            -DBUNGEE_SELF_TEST=0 \
            -Deigen_assert=BUNGEE_ASSERT1 \
            -DEIGEN_DONT_PARALLELIZE=1 \
            '-DBUNGEE_VERSION="0.0.0"' \
            -c "$src" -o "$obj"
    fi
done
# --- Step 3: Create static archive ---
echo "Creating libbungee.a..."
${CROSS_PREFIX}ar rcs build/bungee/libbungee.a build/bungee/*.o

# --- Step 4: Compile and link DSP plugin ---
echo "Compiling DSP plugin..."
${CROSS_PREFIX}g++ -Ofast -shared -fPIC -std=c++20 \
    -march=armv8-a -mtune=cortex-a72 \
    -fomit-frame-pointer -fno-stack-protector \
    -DNDEBUG \
    -I"$BUNGEE_DIR" \
    src/dsp/plugin.cpp \
    -o build/dsp.so \
    build/bungee/libbungee.a \
    -lm

# --- Step 5: Package ---
echo "Packaging..."
rm -rf "dist/$MODULE_ID"
mkdir -p "dist/$MODULE_ID"
cp src/module.json "dist/$MODULE_ID/"
cp src/ui.js "dist/$MODULE_ID/"
cp build/dsp.so "dist/$MODULE_ID/"
[ -f src/help.json ] && cp src/help.json "dist/$MODULE_ID/"
chmod +x "dist/$MODULE_ID/dsp.so"

# Create tarball for release
cd dist
tar -czvf "$MODULE_ID-module.tar.gz" "$MODULE_ID/"
cd ..

echo ""
echo "=== Build Complete ==="
echo "Output: dist/$MODULE_ID/"
echo "Tarball: dist/$MODULE_ID-module.tar.gz"
echo ""
echo "To install on Move:"
echo "  ./scripts/install.sh"
