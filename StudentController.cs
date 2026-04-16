using System;
using System.IO;
using System.Collections.Generic;

namespace PULSR_3
{
    public class StudentController
    {
        // Robot Constants (Do not modify)
        private const double L1 = 26.0;
        private const double L2 = 52.0;
        private const double SCALER = 25.0;
        private const double OFFSET_ANGLE = 20.0;

        // TASK 2: Store the loaded motor commands here
        private List<int> upperCommands;
        private List<int> lowerCommands;

        public StudentController()
        {
            // Initialize lists
            upperCommands = new List<int>();
            lowerCommands = new List<int>();

            // Load trajectory files
            LoadTrajectoryFiles();
        }

        /// <summary>
        /// TASK 1: Forward Kinematics
        /// </summary>
        public int[] ForwardKinematics(double theta1, double theta2)
        {
            // Step 1: Convert to radians
            double t1_rad = ToRadians(theta1);
            double t2_rad = ToRadians(theta2);

            // Step 2: Compute link contributions
            double e2 = L1 * Math.Cos(t1_rad) + L2 * Math.Cos(t2_rad);
            double e1 = L1 * Math.Sin(t1_rad) + L2 * Math.Sin(t2_rad);

            // Step 3: Apply 20° offset rotation
            double offsetRad = ToRadians(OFFSET_ANGLE);

            double x_screen = (e1 * Math.Cos(offsetRad)) - (e2 * Math.Sin(offsetRad));
            double y_screen = -(e2 * Math.Cos(offsetRad)) - (e1 * Math.Sin(offsetRad));

            // Step 4: Apply scaling
            x_screen *= SCALER;
            y_screen *= SCALER;

            return new int[] { (int)x_screen, (int)y_screen };
        }

        /// <summary>
        /// TASK 2A: Load Trajectory Files
        /// </summary>
        private void LoadTrajectoryFiles()
        {
            try
            {
                // Load upper motor commands
                string[] upperLines = File.ReadAllLines("Nupper_targets.txt");
                foreach (string line in upperLines)
                {
                    if (int.TryParse(line.Trim(), out int value))
                    {
                        upperCommands.Add(value);
                    }
                }

                // Load lower motor commands
                string[] lowerLines = File.ReadAllLines("Nlower_targets.txt");
                foreach (string line in lowerLines)
                {
                    if (int.TryParse(line.Trim(), out int value))
                    {
                        lowerCommands.Add(value);
                    }
                }

                Console.WriteLine($"Loaded {upperCommands.Count} upper commands");
                Console.WriteLine($"Loaded {lowerCommands.Count} lower commands");
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error loading trajectory files: " + ex.Message);
            }
        }

        /// <summary>
        /// TASK 2B: Get Motor Commands
        /// </summary>
        public int[] CalculateControl(int robotX, int robotY, double orbitAngle)
        {
            // Step 1: Compute step index
            int step = (int)(270 - orbitAngle);

            // Step 2: Clamp step to valid range
            if (step < 0) step = 0;
            if (step >= upperCommands.Count) step = upperCommands.Count - 1;

            // Step 3: Get motor commands
            int speedUpper = upperCommands[step];
            int speedLower = lowerCommands[step];

            return new int[] { speedUpper, speedLower };
        }

        // Helper: Convert degrees to radians
        private double ToRadians(double degrees)
        {
            return degrees * Math.PI / 180.0;
        }
    }
}