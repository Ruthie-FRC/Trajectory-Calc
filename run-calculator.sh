#!/bin/bash
# Run the trajectory calculator on test-targets.txt with one command
# Usage: ./run-calculator.sh [max_rpm] [spin_rate]
# Defaults: max_rpm=5000, spin_rate=200

MAX_RPM=${1:-5000}
SPIN_RATE=${2:-200}

echo "Running Trajectory Calculator..."
echo "Max RPM: $MAX_RPM"
echo "Spin Rate: $SPIN_RATE rad/s"
echo ""

mvn exec:java -Dexec.mainClass="frc.robot.trajectory.TurretTrajectoryTester" \
    -Dexec.classpathScope=test \
    -Dexec.args="test-targets.txt $MAX_RPM $SPIN_RATE" \
    -q
