@echo off
REM Run the trajectory calculator on test-targets.txt with one command
REM Usage: run-calculator.bat [max_rpm] [spin_rate]
REM Defaults: max_rpm=5000, spin_rate=200

IF "%1"=="" (
    SET MAX_RPM=5000
) ELSE (
    SET MAX_RPM=%1
)

IF "%2"=="" (
    SET SPIN_RATE=200
) ELSE (
    SET SPIN_RATE=%2
)

echo Running Trajectory Calculator...
echo Max RPM: %MAX_RPM%
echo Spin Rate: %SPIN_RATE% rad/s
echo.

mvn exec:java -Dexec.mainClass="frc.robot.trajectory.TurretTrajectoryTester" -Dexec.classpathScope=test -Dexec.args="test-targets.txt %MAX_RPM% %SPIN_RATE%" -q
