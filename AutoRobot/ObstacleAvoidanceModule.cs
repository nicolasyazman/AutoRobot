using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AutoRobot
{
    public class ObstacleAvoidanceModule
    {
        public Robot Robot { get;}
        
        public ObstacleAvoidanceModule(Robot r)
        {
            Robot = r;
        }
        /// <summary>
        /// Checks if there is an obstacle in the current robot trajectory.
        /// </summary>
        /// <param name="ObstaclesPos">List containing the 2D position of the obstacles in the map.</param>
        /// <param name="AnticipationDistance">Distance where we should anticipate the obstacle presence.</param>
        /// <returns>True if there is an obstacle in the trajectory, false otherwise.</returns>
        public bool IsObstacleInTrajectory(List<Point2D> ObstaclesPos, double AnticipationDistance = 20)
        {
            for (int obsId = 0; obsId < ObstaclesPos.Count; obsId++)
            {
                List<Point2D> FutureRealTrajectory = Robot.PathPlanningModule.GetEstimatedFuturTrajectory(AnticipationDistance);
                List<List<Point2D>> Corridor = GetVehicleCorridor(FutureRealTrajectory);
                for (int i = 0; i < Corridor.Count; i++)
                {
                    if (IsPointInsideRectangle(Corridor[i], ObstaclesPos[obsId]))
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        /// <summary>
        /// Computes a 'corridor' which is a danger zone around the robot's future trajectory.
        /// </summary>
        /// <param name="Traj">List of 2D Points which define the future trajectory of the robot.</param>
        /// <returns>"Matrix" containing for each row 4 2DPoints, each being a corner of the square representing the robot's boundaries.</returns>
        public List<List<Point2D>> GetVehicleCorridor(List<Point2D> Traj)
        {
            List<List<Point2D>> corridor = new List<List<Point2D>>();

            int i;
            for (i = 0; i < Traj.Count - 1; i++)
            {
                double angle = Traj[i].AbsoluteBearing(Traj[i + 1]);
                List<Point2D> vehicleBoundaries = new List<Point2D>();
                double Margin = 0.5;
                double distForward = Robot.VehicleLength + 0.5;
                double distBehind = Robot.VehicleLength + 0.5;

                double distLeft = (Robot.VehicleWidth / 2 + Margin);

                Point2D MidPointForward = new Point2D(Traj[i].X + Math.Cos(angle) * distForward, Traj[i].Y + Math.Sin(angle) * distForward);
                Point2D MidPointBehind = new Point2D(Traj[i].X + Math.Cos(angle + Math.PI) * distForward, Traj[i].Y + Math.Sin(angle + Math.PI) * distForward);

                Point2D ForwardLeft = new Point2D(MidPointForward.X + Math.Cos(angle + Math.PI / 2) * distLeft, MidPointForward.Y + Math.Sin(angle + Math.PI / 2) * distLeft);
                Point2D ForwardRight = new Point2D(MidPointForward.X + Math.Cos(angle - Math.PI / 2) * distLeft, MidPointForward.Y + Math.Sin(angle - Math.PI / 2) * distLeft);
                Point2D BehindLeft = new Point2D(MidPointBehind.X + Math.Cos(angle + Math.PI + Math.PI / 2) * distLeft, MidPointBehind.Y + Math.Sin(angle + Math.PI + Math.PI / 2) * distLeft);
                Point2D BehindRight = new Point2D(MidPointBehind.X + Math.Cos(angle + Math.PI - Math.PI / 2) * distLeft, MidPointBehind.Y + Math.Sin(angle + Math.PI - Math.PI / 2) * distLeft);

                vehicleBoundaries.Add(ForwardLeft);
                vehicleBoundaries.Add(ForwardRight);
                vehicleBoundaries.Add(BehindLeft);
                vehicleBoundaries.Add(BehindRight);
                corridor.Add(vehicleBoundaries);
            }

            return corridor;
        }


        double sign(Point2D p1, Point2D p2, Point2D p3)
        {
            return (p1.X - p3.X) * (p2.Y - p3.Y) - (p2.X - p3.X) * (p1.Y - p3.Y);
        }

        /// <summary>
        /// Checks if a point is inside a triangle defined by 3 vertices.
        /// </summary>
        /// <param name="pt">Point that we wish to check is inside the triangle.</param>
        /// <param name="v1">First vertex of the triangle.</param>
        /// <param name="v2">Second vertex of the triangle.</param>
        /// <param name="v3">Third vertex of the tirangle.</param>
        /// <returns>True if the point is inside the triangle, false otherwise.</returns>
        bool PointInTriangle(Point2D pt, Point2D v1, Point2D v2, Point2D v3)
        {
            bool b1, b2, b3;

            b1 = sign(pt, v1, v2) < 0;
            b2 = sign(pt, v2, v3) < 0;
            b3 = sign(pt, v3, v1) < 0;

            return ((b1 == b2) && (b2 == b3));
        }

        /// <summary>
        /// Checks if a point is inside a rectangle by checking if that point is inside one of the two triangles making up the vehicle.
        /// </summary>
        /// <param name="rectPoints">List of 4 2D Points representing the vertices of the rectangle (boundaries / corners / etc...)</param>
        /// <param name="point">Point we wish to see if it is inside the rectangle.</param>
        /// <returns>True if the point is inside the rectangle, false otherwise.</returns>
        public bool IsPointInsideRectangle(List<Point2D> rectPoints, Point2D point)
        {
            rectPoints.Sort((point1, point2) => point1.Y.CompareTo(point2.Y));
            if (PointInTriangle(point, rectPoints[0], rectPoints[1], rectPoints[2])
                || PointInTriangle(point, rectPoints[0], rectPoints[1], rectPoints[2]))
                return true;
            return false;
        }

    }
}
