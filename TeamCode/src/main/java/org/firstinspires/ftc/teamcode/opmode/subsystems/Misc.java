package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.*;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Date;
import java.util.Locale;

@Config
public class Misc {
    public static boolean logTele = true;
    public static double currentLogIntervalMs = 1000;
    public static double currentDisplayIntervalMs = 250;
    public static int maxCurrentLogs = 5;
    public static String currentLogDir = "/sdcard/FIRST/currentLogs";
    public static double intakeFullCurrent = 6.0;
    public static double ledFullColor = 0.5;
    public static double ledIdleColor = 0.279;

    public HardwareMap hwmap;
    public Limelight3A limelight;
    Robot robot;

    public static final String[] MOTOR_NAMES = {"LF", "RF", "LB", "RB", "leftIntake", "rightIntake", "nectarShooter", "pollenShooter"};
    public double[] currents = new double[MOTOR_NAMES.length];
    public boolean logging = logTele;
    public boolean showCurrentDraw = false;
    public String logStatus = "not started";
    BufferedWriter writer = null;
    ElapsedTime teleTimer = new ElapsedTime();
    ElapsedTime logTimer = new ElapsedTime();
    ElapsedTime displayTimer = new ElapsedTime();

    public ServoImplEx led;
    public boolean intakeFull = false;
    public double intakeCurrent = 0;
    double lastLedPosition = -1;

    public Misc(HardwareMap hwmap){
        this.hwmap = hwmap;
    }

    public Misc(HardwareMap hwmap, Robot robot){
        this.hwmap = hwmap;
        this.robot = robot;
    }

    public boolean canShoot(String alliance){
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (alliance.equalsIgnoreCase("red") && id >= 30 && id <= 37) return true;
            if (alliance.equalsIgnoreCase("blue") && id >= 38 && id <= 45) return true;
        }
        return false;
    }

    DcMotorEx[] motors(){
        return new DcMotorEx[]{robot.dt.LF, robot.dt.RF, robot.dt.LB, robot.dt.RB,
                robot.intake.leftIntake, robot.intake.rightIntake,
                robot.shooter.nectarShooter, robot.shooter.pollenShooter};
    }

    void readCurrents(){
        DcMotorEx[] motors = motors();
        for (int i = 0; i < motors.length; i++) {
            currents[i] = motors[i].getCurrent(CurrentUnit.AMPS);
        }
    }

    public void startLogging(){
        if(!logging || robot == null){
            logStatus = "disabled";
            return;
        }
        try{
            File dir = new File(currentLogDir);
            if(!dir.exists() && !dir.mkdirs()){
                throw new IOException("could not create " + currentLogDir);
            }
            File[] old = dir.listFiles((d, name) -> name.endsWith(".csv"));
            if(old != null){
                Arrays.sort(old, Comparator.comparingLong(File::lastModified));
                for (int i = 0; i <= old.length - maxCurrentLogs; i++) {
                    old[i].delete();
                }
            }
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            File file = new File(dir, "current_" + stamp + ".csv");
            writer = new BufferedWriter(new FileWriter(file));
            StringBuilder header = new StringBuilder("time_s");
            for (String name : MOTOR_NAMES) {
                header.append(',').append(name).append("_amps");
            }
            header.append(",nectar_distance_from_goal,nectar_flywheel_target_velocity");
            header.append(",pollen_distance_from_goal,pollen_flywheel_target_velocity");
            writer.write(header.toString());
            writer.newLine();
            writer.flush();
            logStatus = "logging to " + file.getName();
        }catch(IOException e){
            writer = null;
            logStatus = "error: " + e.getMessage();
        }
        teleTimer.reset();
        logTimer.reset();
    }

    void writeLogLine(){
        StringBuilder line = new StringBuilder(String.format(Locale.US, "%.2f", teleTimer.seconds()));
        for (double amps : currents) {
            line.append(',').append(String.format(Locale.US, "%.3f", amps));
        }
        line.append(',').append(String.format(Locale.US, "%.2f", robot.shooter.targetDistNectar));
        line.append(',').append(String.format(Locale.US, "%.1f", robot.shooter.targetVelocityNectar));
        line.append(',').append(String.format(Locale.US, "%.2f", robot.shooter.targetDistPollen));
        line.append(',').append(String.format(Locale.US, "%.1f", robot.shooter.targetVelocityPollen));
        try{
            writer.write(line.toString());
            writer.newLine();
            writer.flush();
        }catch(IOException e){
            logStatus = "error: " + e.getMessage();
            closeLog();
        }
    }

    void updateLed(){
        if(led == null){
            return;
        }
        if(!robot.intake.running || robot.intake.outtaking){
            intakeFull = false;
        }else{
            intakeCurrent = Math.max(robot.intake.leftIntake.getCurrent(CurrentUnit.AMPS),
                    robot.intake.rightIntake.getCurrent(CurrentUnit.AMPS));
            intakeFull = intakeCurrent >= intakeFullCurrent;
        }
        double position = intakeFull ? ledFullColor : ledIdleColor;
        if(position != lastLedPosition){
            led.setPosition(position);
            lastLedPosition = position;
        }
    }

    void closeLog(){
        if(writer == null){
            return;
        }
        try{
            writer.close();
        }catch(IOException ignored){
        }
        writer = null;
    }

    public void init(){
        limelight = hwmap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        logging = logTele;
        led = hwmap.tryGet(ServoImplEx.class, "light");
        if(led != null){
            led.setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        intakeFull = false;
        lastLedPosition = -1;
    }

    public void initLoop(){

    }

    public void start(){
        limelight.start();
    }

    public void stop(){
        limelight.stop();
        closeLog();
    }

    public void update(){
        if(robot == null){
            return;
        }
        updateLed();
        boolean logDue = writer != null && logTimer.milliseconds() >= currentLogIntervalMs;
        boolean displayDue = showCurrentDraw && displayTimer.milliseconds() >= currentDisplayIntervalMs;
        if(logDue || displayDue){
            readCurrents();
            displayTimer.reset();
        }
        if(logDue){
            writeLogLine();
            logTimer.reset();
        }
    }
}
