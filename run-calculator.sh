#!/bin/bash
# Run the trajectory calculator on test-targets.txt with one command
# Usage: ./run-calculator.sh [max_speed_ms] [spin_rate]
# Defaults: max_speed_ms=30, spin_rate=200

MAX_SPEED_MS=${1:-30}
SPIN_RATE=${2:-200}

echo "Running Trajectory Calculator..."
echo "Max Ball Speed: $MAX_SPEED_MS m/s"
echo "Spin Rate: $SPIN_RATE rad/s"
echo ""

mvn exec:java -Dexec.mainClass="frc.robot.trajectory.TurretTrajectoryTester" \
    -Dexec.classpathScope=test \
    -Dexec.args="test-targets.txt $MAX_SPEED_MS $SPIN_RATE" \
    -q
