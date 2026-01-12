@echo off
REM Run the trajectory calculator on test-targets.txt with one command
REM Usage: run-calculator.bat [max_speed_fps] [spin_rate]
REM Defaults: max_speed_fps=30, spin_rate=200

IF "%1"=="" (
    SET MAX_SPEED_FPS=30
) ELSE (
    SET MAX_SPEED_FPS=%1
)

IF "%2"=="" (
    SET SPIN_RATE=200
) ELSE (
    SET SPIN_RATE=%2
)

echo Running Trajectory Calculator...
echo Max Ball Speed: %MAX_SPEED_FPS% ft/s
echo Spin Rate: %SPIN_RATE% rad/s
echo.

mvn exec:java -Dexec.mainClass="frc.robot.trajectory.TurretTrajectoryTester" -Dexec.classpathScope=test -Dexec.args="test-targets.txt %MAX_SPEED_FPS% %SPIN_RATE%" -q
