// Copyright (c) Choreo contributors

package org.firstinspires.ftc.teamcode.Choreo;


import com.google.gson.Gson;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
/** Utilities to load and follow ChoreoTrajectories */
public class Choreo {
    private static final Gson gson = new Gson();

    /** Default constructor. */
    public Choreo() {}

    /**
     * Load a trajectory from the deploy directory. Choreolib expects .traj files to be placed in
     * src/main/deploy/choreo/[trajName].traj .
     *
     * @param trajName the path name in Choreo, which matches the file name in the deploy directory.
     *     Do not include ".traj" here.
     * @return the loaded trajectory, or null if the trajectory could not be loaded.
     */
    public static ChoreoTrajectory getTrajectory(String trajName) {
        File traj_dir = new File("java/org/firstinspires/ftc/teamcode/Trajectories");
        File traj_file = new File(traj_dir, trajName + ".traj");

        return loadFile(traj_file);
    }

    /**
     * Loads the split parts of the specified trajectory. Fails and returns null if any of the parts
     * could not be loaded.
     *
     * <p>This method determines the number of parts to load by counting the files that match the
     * pattern "trajName.X.traj", where X is a string of digits. Let this count be N. It then attempts
     * to load "trajName.1.traj" through "trajName.N.traj", consecutively counting up. If any of these
     * files cannot be loaded, the method returns null.
     *
     * @param trajName The path name in Choreo for this trajectory.
     * @return The ArrayList of segments, in order, or null.
     */
    public static ArrayList<ChoreoTrajectory> getTrajectoryGroup(String trajName) {
        // Count files matching the pattern for split parts.
        File traj_dir = new File("java/org/firstinspires/ftc/teamcode/Trajectories");
        File[] files =
                traj_dir.listFiles((file) -> file.getName().matches(trajName + "\\.\\d+\\.traj"));
        assert files != null;
        int segmentCount = files.length;
        // Try to load the segments.
        ArrayList<ChoreoTrajectory> trajs = new ArrayList<ChoreoTrajectory>();
        for (int i = 1; i <= segmentCount; ++i) {
            File traj = new File(traj_dir, String.format("%s.%d.traj", trajName, i));
            ChoreoTrajectory trajectory = loadFile(traj);
            if (trajectory == null) {
                return null;
            }
            trajs.add(trajectory);
        }

        return trajs;
    }

    private static ChoreoTrajectory loadFile(File path) {
        try {
            BufferedReader reader = new BufferedReader(new FileReader(path));
            ChoreoTrajectory traj = gson.fromJson(reader, ChoreoTrajectory.class);

            return traj;
        } catch (Exception ex) {
            return null;
        }

    }



}