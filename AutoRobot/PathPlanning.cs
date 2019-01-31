using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AutoRobot
{
    public class PathPlanning
    {
        Robot Robot { get; }

        public PathPlanning(Robot r)
        {
            Robot = r;
        }

        public double ComputeSteeringToFollowTrajectory()
        {
            int closestIdx;
            List<Point2D> segment = Robot.Traj.FindTrajectorySegment(Robot, out closestIdx);
            Robot.MinIdxTraj = (int)Math.Max(closestIdx, Robot.MinIdxTraj);

            double angle = segment[0].AbsoluteBearing(segment[1]);

            Robot.CurvilinearAbscissa += Math.Abs(Robot.Speed);

            double distanceTol = 0.1;
            Point2D closest;
            double distanceToTrajSegment = Robot.Traj.FindDistanceToSegment(Robot.Position, segment[0], segment[1], out closest);

            double PsiConsigne;

            if (distanceToTrajSegment < distanceTol)
            {
                PsiConsigne = Geometry.Delta_Bearing(Robot.Bearing * 180 / Math.PI, angle * 180 / Math.PI);
                PsiConsigne *= Math.PI / 180;
            }
            else
            {
                double correctionAngle = 1000000;
                double maxDeltaDistance = 10;
                double firstIdx = closestIdx;
                double minAngle = 100000;
                double distCumul = 0;
                double lookForward = 5;
                while (closestIdx < Robot.Traj.TrajectoryPoints.Count - 3 && distCumul < lookForward)
                    distCumul += Robot.Traj.TrajectoryPoints[closestIdx].Norm(Robot.Traj.TrajectoryPoints[closestIdx++ + 1]);

                while (distCumul < maxDeltaDistance && closestIdx < Robot.Traj.TrajectoryPoints.Count - 1) //&& (correctionAngle > correctionAngleTol || 2 * Math.PI - correctionAngle > correctionAngleTol))
                {
                    correctionAngle = Robot.Position.AbsoluteBearing(Robot.Traj.TrajectoryPoints[closestIdx]);
                    if ((correctionAngle + Math.PI * 2) % Math.PI < minAngle)
                        minAngle = (correctionAngle + Math.PI * 2) % (2 * Math.PI);
                    distCumul += Robot.Traj.TrajectoryPoints[closestIdx].Norm(Robot.Traj.TrajectoryPoints[closestIdx++ + 1]);
                }
                PsiConsigne = Geometry.Delta_Bearing(Robot.Bearing * 180 / Math.PI, minAngle * 180 / Math.PI);
                PsiConsigne *= Math.PI / 180;
            }

            return PsiConsigne;
        }
        /// <summary>
        /// Computes the estimated future trajectory the robot will follow.
        /// </summary>
        /// <param name="estimatedDistance">Estimation distance in meters.</param>
        /// <returns>List of 2D Points, each of them being a position the robot will occupy in the future.</returns>
        public List<Point2D> GetEstimatedFuturTrajectory(double estimatedDistance = 20)
        {
            estimatedDistance *= (Robot.Speed + 2);
            List<Point2D> futurTrajectory = new List<Point2D>();
            Point2D actualPosition = new Point2D(Robot.Position.X, Robot.Position.Y);
            Robot estimatedRobot = new Robot(actualPosition);
            estimatedRobot.Traj = new Trajectory(Robot.Traj.TrajectoryPoints);
            estimatedRobot.Speed = Robot.Speed;
            estimatedRobot.Fd = 10;
            estimatedRobot.Bearing = Robot.Bearing;
            estimatedRobot.MinIdxTraj = Robot.MinIdxTraj;
            estimatedRobot.InputVoltage1 = Robot.InputVoltage1;
            estimatedRobot.SteeringVoltage = Robot.SteeringVoltage;
            estimatedRobot.Psi = Robot.Psi;
            futurTrajectory.Add(actualPosition);
            double distanceCumulee = 0;

            while (distanceCumulee < estimatedDistance && estimatedRobot.MinIdxTraj < estimatedRobot.Traj.TrajectoryPoints.Count)
            {
                double RegulatedSpeedVoltage;
                double RegulatedSteeringVoltage;
                estimatedRobot.MoveRobot(out RegulatedSpeedVoltage, out RegulatedSteeringVoltage, 0.6, 0.6);
                futurTrajectory.Add(estimatedRobot.Position);
                distanceCumulee += futurTrajectory[futurTrajectory.Count - 1].
                    Norm(futurTrajectory[futurTrajectory.Count - 2]);
            }

            return futurTrajectory;
        }

        public int[,] CreatePotentialField(List<Point2D> ObstaclesGC, double ObstacleSize, out int VehGridX, out int VehGridY, double GridWidth = 100, double GridHeight = 100, double SafetyDistance = 10, double LookIdx = 10)
        {
            // TO TEST THINGS
            ObstacleSize += SafetyDistance * 2;
            double GridResolution = 0.25;

            int NumberOfCellsPerRow = (int)Math.Ceiling(GridWidth / GridResolution);
            int NumberOfRows = (int)Math.Ceiling(GridHeight / GridResolution);

            int[,] Grid = new int[NumberOfRows, NumberOfCellsPerRow];

            double GridStartX = NumberOfCellsPerRow / 2;
            double GridStartY = NumberOfRows / 2;

            double LookForwardDistanceMeters = 15;
            int LookForward = 10;
            double CurvAbsCumul = 0;

            while (CurvAbsCumul < LookForwardDistanceMeters && Robot.MinIdxTraj + LookForward < Robot.Traj.TrajectoryPoints.Count - 2)
            {
                CurvAbsCumul += Robot.Traj.TrajectoryPoints[Robot.MinIdxTraj + LookForward].Norm(Robot.Traj.TrajectoryPoints[Robot.MinIdxTraj + LookForward + 1]);
                LookForward++;
            }
            int minidx;

            Robot.Traj.FindTrajectorySegment(Robot, out minidx, 100000);
            for (int i = (int)(Math.Min(minidx + LookForward, Robot.Traj.TrajectoryPoints.Count)); i < (int)(Math.Min(Robot.Traj.TrajectoryPoints.Count, Robot.MinIdxTraj + LookForward + LookIdx)); i++)
            {
                Point2D trajPoint = Robot.Traj.TrajectoryPoints[i];
                int Gridx = (int)(trajPoint.X / GridResolution + GridStartX);
                int Gridy = (int)(trajPoint.Y / GridResolution + GridStartY);
                if (Gridx >= 0 && Gridx < NumberOfCellsPerRow && Gridy >= 0 && Gridy < NumberOfRows)
                    Grid[Gridy, Gridx] = -3;
            }


            for (int i = 0; i < ObstaclesGC.Count; i++)
            {
                double obstacleX = ObstaclesGC[i].X / GridResolution + GridStartX;
                double obstacleY = ObstaclesGC[i].Y / GridResolution + GridStartY;
                int StartX = Math.Max(0, (int)(obstacleX - (ObstacleSize / GridResolution)));
                int EndX = Math.Min(NumberOfCellsPerRow - 1, (int)(obstacleX + (ObstacleSize / GridResolution)));
                int StartY = Math.Max(0, (int)(obstacleY - ObstacleSize / GridResolution));
                int EndY = Math.Min(NumberOfRows - 1, (int)(obstacleY + ObstacleSize / GridResolution));
                for (int y = StartY; y < EndY; y++)
                {
                    for (int x = StartX; x < EndX; x++)
                    {
                        if (new Point2D(x, y).Norm(new Point2D(obstacleX, obstacleY)) < (SafetyDistance / GridResolution))
                        {
                            Grid[y, x] = -1;
                        }
                    }
                }
            }

            VehGridX = (int)Math.Floor(Robot.Position.X / GridResolution + GridStartX);
            VehGridY = (int)Math.Floor(Robot.Position.Y / GridResolution + GridStartY);
            return Grid;
        }


        /// <summary>
        /// This function returns the shortest path between the robot starting postion defined with StartX and StartY coordinates and a 'goal point'.
        /// </summary>
        /// <param name="Grid">Two dimensional int array with a width of GridWidth meters and a height of Gridheight.</param>
        /// <param name="GridWidth">Width of the grid in meters.</param>
        /// <param name="GridHeight">Height of the grid in meters.</param>
        /// <param name="StartX">Starting X of the pathfinding (robot X position).</param>
        /// <param name="StartY">Starting Y of the pathfinding (robot Y position).</param>
        /// <param name="GridResolution">Resolution of the grid, meaning for 1 real life meter there are 1 / GridResolution grid cells. Default value is 0.25 (4 grid cells per real-life meter.)</param>
        /// <returns>A List of Point2D containing the subsequent positions from the origin to the first 'goal point' reached.</returns>
        public List<Point2D> FindShortestPath(int[,] Grid, int GridWidth, int GridHeight, int StartX, int StartY, double GridResolution = 0.25)
        {
            GridWidth = (int)(GridWidth / GridResolution);
            GridHeight = (int)(GridHeight / GridResolution);


            double GridStartX = GridWidth / 2;
            double GridStartY = GridHeight / 2;
            Queue<Tuple<int, int, int>> queue = new Queue<Tuple<int, int, int>>();

            queue.Enqueue(new Tuple<int, int, int>(StartX, StartY, 0));
            int X = 0, Y = 0;
            int Val = 0;
            int MaximumVal = 0;
            while (!(queue.Count == 0) && Val > -2)
            {
                Tuple<int, int, int> Position = queue.Dequeue();

                X = Position.Item1;
                Y = Position.Item2;

                if (X < 0 || X >= GridWidth || Y < 0 || Y >= GridHeight)
                    continue;
                Val = Position.Item3;
                if (Val > MaximumVal)
                    MaximumVal = Val;
                if (Val > -1) // Destination not found
                {
                    // West
                    if (X - 1 >= 0 && Grid[Y, X - 1] != -1)
                    {
                        if (Grid[Y, X - 1] == 0)
                        {
                            Grid[Y, X - 1] = Val + 1;
                            queue.Enqueue(new Tuple<int, int, int>(X - 1, Y, Val + 1));
                        }
                        else if (Grid[Y, X - 1] < -1)
                        {
                            queue.Enqueue(new Tuple<int, int, int>(X - 1, Y, -2));
                        }

                    }
                    // North West
                    if (X - 1 >= 0 && Y - 1 >= 0 && Grid[Y - 1, X - 1] != -1)
                    {
                        if (Grid[Y - 1, X - 1] == 0)
                        {
                            Grid[Y - 1, X - 1] = Val + 1;
                            queue.Enqueue(new Tuple<int, int, int>(X - 1, Y - 1, Val + 1));
                        }
                        else if (Grid[Y - 1, X - 1] < -1)
                        {
                            queue.Enqueue(new Tuple<int, int, int>(X - 1, Y - 1, -2));
                        }
                    }
                    // East
                    if (X + 1 < GridWidth && Grid[Y, X + 1] != -1)
                    {
                        if (Grid[Y, X + 1] == 0)
                        {
                            Grid[Y, X + 1] = Val + 1;
                            queue.Enqueue(new Tuple<int, int, int>(X + 1, Y, Val + 1));
                        }
                        else if (Grid[Y, X + 1] < -1)
                        {
                            queue.Enqueue(new Tuple<int, int, int>(X + 1, Y, -2));
                        }
                    }
                    // North East
                    if (X + 1 < GridWidth && Y - 1 >= 0 && Grid[Y - 1, X + 1] != -1)
                    {
                        if (Grid[Y - 1, X + 1] == 0)
                        {
                            Grid[Y - 1, X + 1] = Val + 1;
                            queue.Enqueue(new Tuple<int, int, int>(X + 1, Y - 1, Val + 1));
                        }
                        else if (Grid[Y - 1, X + 1] < -1)
                        {
                            queue.Enqueue(new Tuple<int, int, int>(X + 1, Y - 1, -2));
                        }
                    }

                    // North
                    if (Y - 1 >= 0 && Grid[Y - 1, X] != -1)
                    {
                        if (Grid[Y - 1, X] == 0)
                        {
                            Grid[Y - 1, X] = Val + 1;
                            queue.Enqueue(new Tuple<int, int, int>(X, Y - 1, Val + 1));
                        }
                        else if (Grid[Y - 1, X] < -1)
                        {
                            queue.Enqueue(new Tuple<int, int, int>(X, Y - 1, -2));
                        }
                    }

                    // South
                    if (Y + 1 < GridHeight && Grid[Y + 1, X] != -1)
                    {
                        if (Grid[Y + 1, X] == 0)
                        {
                            Grid[Y + 1, X] = Val + 1;
                            queue.Enqueue(new Tuple<int, int, int>(X, Y + 1, Val + 1));
                        }
                        else if (Grid[Y + 1, X] < -1)
                        {
                            queue.Enqueue(new Tuple<int, int, int>(X, Y + 1, -2));
                        }
                    }

                    // South West
                    if (Y + 1 < GridHeight && X - 1 >= 0 && Grid[Y + 1, X - 1] != -1)
                    {
                        if (Grid[Y + 1, X - 1] == 0)
                        {
                            Grid[Y + 1, X - 1] = Val + 1;
                            queue.Enqueue(new Tuple<int, int, int>(X - 1, Y + 1, Val + 1));
                        }
                        else if (Grid[Y + 1, X - 1] < -1)
                        {
                            queue.Enqueue(new Tuple<int, int, int>(X - 1, Y + 1, -2));
                        }
                    }
                    // South East
                    if (Y + 1 < GridHeight && X + 1 < GridWidth && Grid[Y + 1, X + 1] != -1)
                    {
                        if (Grid[Y + 1, X + 1] == 0)
                        {
                            Grid[Y + 1, X + 1] = Val + 1;
                            queue.Enqueue(new Tuple<int, int, int>(X + 1, Y + 1, Val + 1));
                        }
                        else if (Grid[Y + 1, X + 1] < -1)
                        {
                            queue.Enqueue(new Tuple<int, int, int>(X + 1, Y + 1, -2));
                        }
                    }
                }
                else if (Val < -1)
                {
                    Grid[Y, X] = MaximumVal + 1;
                }
            }

            if (Val != -2) // Not found objective
            {
                return null;
            }

            Val = Grid[Y, X];
            List<Point2D> ShortestPath = new List<Point2D>();
            //ShortestPath.Add(new Point2D(X, Y));
            int ValShouldBe = Val;
            // Now do the inverse way
            while (ValShouldBe > 0)
            {
                // West
                if (X - 1 >= 0 && Grid[Y, X - 1] == ValShouldBe)
                {
                    X = X - 1;
                    ShortestPath.Add(new Point2D((X - GridStartX) * GridResolution, (Y - GridStartY) * GridResolution));
                }
                // East
                else if (X + 1 < GridWidth && Grid[Y, X + 1] == ValShouldBe)
                {
                    X = X + 1;
                    ShortestPath.Add(new Point2D((X - GridStartX) * GridResolution, (Y - GridStartY) * GridResolution));
                }
                // North
                else if (Y - 1 >= 0 && Grid[Y - 1, X] == ValShouldBe)
                {

                    Y = Y - 1;
                    ShortestPath.Add(new Point2D((X - GridStartX) * GridResolution, (Y - GridStartY) * GridResolution));
                }
                // South
                else if (Y + 1 < GridHeight && Grid[Y + 1, X] == ValShouldBe)
                {
                    Y = Y + 1;
                    ShortestPath.Add(new Point2D((X - GridStartX) * GridResolution, (Y - GridStartY) * GridResolution));
                }
                // South West
                else if (Y + 1 < GridHeight && X - 1 >= 0 && Grid[Y + 1, X - 1] == ValShouldBe)
                {
                    Y = Y + 1;
                    X = X - 1;
                    ShortestPath.Add(new Point2D((X - GridStartX) * GridResolution, (Y - GridStartY) * GridResolution));

                }
                // South East
                else if (Y + 1 < GridHeight && X + 1 < GridWidth && Grid[Y + 1, X + 1] == ValShouldBe)
                {
                    Y = Y + 1;
                    X = X + 1;
                    ShortestPath.Add(new Point2D((X - GridStartX) * GridResolution, (Y - GridStartY) * GridResolution));
                }

                // North West
                else if (Y - 1 >= 0 && X - 1 >= 0 && Grid[Y - 1, X - 1] == ValShouldBe)
                {

                    Y = Y - 1;
                    X = X - 1;
                    ShortestPath.Add(new Point2D((X - GridStartX) * GridResolution, (Y - GridStartY) * GridResolution));
                }

                // North East
                else if (Y - 1 >= 0 && X + 1 < GridWidth && Grid[Y - 1, X + 1] == ValShouldBe)
                {

                    Y = Y - 1;
                    X = X + 1;
                    ShortestPath.Add(new Point2D((X - GridStartX) * GridResolution, (Y - GridStartY) * GridResolution));
                }
                ValShouldBe--;
            }
            ShortestPath.Reverse();

            return ShortestPath;
        }
    }
}
