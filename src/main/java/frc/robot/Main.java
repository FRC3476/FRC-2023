// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

import java.lang.management.ManagementFactory;
import java.lang.management.RuntimeMXBean;
import java.util.List;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what you are doing, do not modify
 * this file except to change the parameter class to the startRobot call.
 */
public final class Main {
    private Main() {}

    /**
     * Main initialization method. Do not perform any initialization here.
     * <p>
     * If you change your main Robot class (name), change the parameter type.
     */
    public static void main(String... args) {
        doPreInit();
        RobotBase.startRobot(Robot::new);
    }

    private static final String USING_MODIFIABLE_JVM_ARGS_FLAG = "-Dusing.modified.args=true";

    // The pracice bot has very limited memory, so we need to limit the heap size, but G1GC is more efficient with larger heaps
    // so we use a larger heap size on the competition bot (which has more memory)
    private static final int MAX_JAVA_HEAP_SIZE_MB = Constants.IS_PRACTICE ? 12 : 50;
    private static final List<String> WANTED_ARGS = List.of(

            // JMX remote debugging
            "-Dcom.sun.management.jmxremote=true",
            "-Dcom.sun.management.jmxremote.port=1198",
            "-Dcom.sun.management.jmxremote.local.only=false",
            "-Dcom.sun.management.jmxremote.ssl=false",
            "-Dcom.sun.management.jmxremote.authenticate=false",
            "-Djava.rmi.server.hostname=10.34.76.2",

            "-XX:+UnlockExperimentalVMOptions",
            "-Xmx" + MAX_JAVA_HEAP_SIZE_MB + "M",
            "-Xms" + MAX_JAVA_HEAP_SIZE_MB + "M", // Set the minimum heap size to the maximum heap size to avoid resizing
            "-Djoml.fastmath=false", // Disable fast math for JOML (it causes all sorts of weird issues)
            "-ea", // Enable assertions
            "-XX:+UseG1GC",
            "-XX:MaxGCPauseMillis=4", // Higher than the default 1ms, but allows for more consistent performance
            "-XX:+AlwaysPreTouch", // Pre-touch memory pages used by the JVM during initialization
            "-XX:+ParallelRefProcEnabled", // Use multiple threads to clean up reference objects
            // Causes GC to write to file system which can cause major latency if disk IO is high -- See https://www.evanjones.ca/jvm-mmap-pause.html
            "-XX:+PerfDisableSharedMem",
            "-Xlog:gc*:logs/gc.log:time,uptime:filecount=5,filesize=1M", // Log GC events to a file
            USING_MODIFIABLE_JVM_ARGS_FLAG
    );

    private static void doPreInit() {
        RuntimeMXBean runtimeMxBean = ManagementFactory.getRuntimeMXBean();
        List<String> arguments = runtimeMxBean.getInputArguments();
        if (arguments.contains(USING_MODIFIABLE_JVM_ARGS_FLAG)) {
            // We are already using the modified arguments, so we don't need to do anything
            System.out.println("Modified JVM args already in use; continuing");
            return;
        }

        if (!arguments.contains("-Don.robot=true")) {
            // We are not running on the robot, so we don't need to modify the JVM args
            // (this is useful for debugging)
            System.out.println("Not running on the robot; continuing");
            return;
        }

        // We are not using the modified arguments, so we need to restart the JVM with the modified arguments
        String javaHome = System.getProperty("java.home");
        String javaBin = javaHome + "/bin/java";
        String classpath = System.getProperty("java.class.path");
        String mainClass = Main.class.getCanonicalName();
        String jvmArgs = String.join(" ", WANTED_ARGS);
        String command = String.format("%s %s -cp %s %s", javaBin, jvmArgs, classpath, mainClass);
        System.out.println("Restarting with modified JVM args: " + command);
        try {
            ProcessBuilder builder = new ProcessBuilder(command.split(" "));
            // TODO: Check that this properly redirects the output to the console
            builder.redirectErrorStream(true);
            builder.redirectOutput(ProcessBuilder.Redirect.INHERIT);
            builder.redirectInput(ProcessBuilder.Redirect.INHERIT);
            builder.start();
            System.exit(0);
        } catch (Exception e) {
            throw new RuntimeException("Failed to restart with modified JVM args", e);
        }
    }
}
