using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AutoRobot
{
    public class Trajectory
    {
        public List<Point2D> TrajectoryPoints { get; set; }

        public Trajectory(List<Point2D> TrajPoints)
        {
            TrajectoryPoints = TrajPoints;
        }


        /// <summary>
        /// Not used ?
        /// </summary>
        /// <param name="endIndex"></param>
        /// <returns></returns>
        /*public double CurvilinearAbscissaCumul(int endIndex)
        {
            double cumul = 0;
            if (endIndex >= Trajectory.Count)
                endIndex = Trajectory.Count - 1;
            for (int i = 0; i < endIndex; i++)
            {
                cumul += Trajectory[i].Norm(Trajectory[i + 1]);
            }
            return cumul;
        }*/

        /// <summary>
        /// This function calculates the euclidian distance between a 2D Point and a segment defined by two points. Returns the distance and the closest point. 
        /// </summary>
        /// <param name="pt">2D Point whose euclidian distance to the segment p1->p2 we want to compute.</param>
        /// <param name="p1">First point of the segment.</param>
        /// <param name="p2">Second point of the segment.</param>
        /// <param name="closest">Output parameter. Closest point to the segment.</param>
        /// <returns>Euclidian distance in meters from the point pt to the closest point.</returns>
        public double FindDistanceToSegment(Point2D pt, Point2D p1, Point2D p2, out Point2D closest)
        {
            double dx = p2.X - p1.X;
            double dy = p2.Y - p1.Y;
            if ((dx == 0) && (dy == 0))
            {
                // It's a point not a line segment.
                closest = p1;
                dx = pt.X - p1.X;
                dy = pt.Y - p1.Y;
                return Math.Sqrt(dx * dx + dy * dy);
            }

            // Calculate the t that minimizes the distance.
            double t = ((pt.X - p1.X) * dx + (pt.Y - p1.Y) * dy) /
                (dx * dx + dy * dy);

            // See if this represents one of the segment's
            // end points or a point in the middle.
            if (t < 0)
            {
                closest = new Point2D(p1.X, p1.Y);
                dx = pt.X - p1.X;
                dy = pt.Y - p1.Y;
            }
            else if (t > 1)
            {
                closest = new Point2D(p2.X, p2.Y);
                dx = pt.X - p2.X;
                dy = pt.Y - p2.Y;
            }
            else
            {
                closest = new Point2D(p1.X + t * dx, p1.Y + t * dy);
                dx = pt.X - closest.X;
                dy = pt.Y - closest.Y;
            }

            return Math.Sqrt(dx * dx + dy * dy);
        }

        /// <summary>
        /// Computes a number nbPoints of intermediate points between two points which define a segment.
        /// </summary>
        /// <param name="origin">Start of the segment.</param>
        /// <param name="dest">End of the segment.</param>
        /// <param name="nbPoints">Number of points.</param>
        /// <returns>List of interpolated 2D points.</returns>
        public List<Point2D> CalculateIntermediatePointsBetween2Points(Robot r, Point2D origin, Point2D dest, double nbPoints)
        {

            double distance = origin.Norm(dest);
            double distanceBetweenPoints = distance / (nbPoints + 1);
            double angle = origin.AbsoluteBearing(dest);

            List<Point2D> res = new List<Point2D>();
            for (int i = 0; i < nbPoints; i++)
            {
                Point2D pointIntermediate = r.CalculateRobotNextPositionPolar(origin, angle, (distanceBetweenPoints * (i + 1)));
                res.Add(pointIntermediate);
            }

            return res;
        }

        /// <summary>
        /// Computes a number N of interpolated points between two points which define a segment. These points are separated by distanceBetweenPoints.
        /// </summary>
        /// <param name="origin">Start of the segment.</param>
        /// <param name="dest">End of the segment.</param>
        /// <param name="distanceBetweenPoints">Distance between the points, in meters.</param>
        /// <param name="accumulatedDistance">Accumulated distance, in meters.</param>
        /// <returns></returns>
        public List<Point2D> CalculateIntermediatePointsSeparatedByDistance(Robot r, Point2D origin, Point2D dest, double distanceBetweenPoints, double accumulatedDistance = 0)
        {
            double totalDistance = origin.Norm(dest);
            double numberPoints = (int)Math.Floor((totalDistance) / distanceBetweenPoints);
            double angle = origin.AbsoluteBearing(dest);

            List<Point2D> res = new List<Point2D>();
            int i = 0;
            while ((i) * distanceBetweenPoints + accumulatedDistance < totalDistance)
            {
                Point2D pointIntermediate = r.CalculateRobotNextPositionPolar(origin, angle, (distanceBetweenPoints * (i) + accumulatedDistance));
                res.Add(pointIntermediate);
                i++;
            }

            return res;
        }

        /// <summary>
        /// Takes a list of WayPoints as an input and returns a set of points separated by distanceBetweenPoints which are interpolated between these waypoints.
        /// </summary>
        /// <param name="WayPoints">Interpolation points.</param>
        /// <param name="distanceBetweenPoints">Distance between the points in meters.</param>
        /// <returns>List of interpolated 2D Points.</returns>
        public List<Point2D> CalculateIntermediatePointsBetweenWayPoints(Robot r, List<Point2D> WayPoints, double distanceBetweenPoints)
        {
            List<Point2D> totalPoints = new List<Point2D>();
            double distanceRestante = 0;
            for (int i = 0; i < WayPoints.Count - 1; i++)
            {
                List<Point2D> currentPoints = CalculateIntermediatePointsSeparatedByDistance(r, WayPoints[i], WayPoints[i + 1], distanceBetweenPoints, distanceRestante);

                totalPoints.AddRange(currentPoints);
                double distanceParcourue = 0;
                for (int j = 0; j < currentPoints.Count - 1; j++)
                    distanceParcourue += currentPoints[j].Norm(currentPoints[j + 1]);

                distanceParcourue += distanceRestante;
                double lengthWayPointLine = WayPoints[i].Norm(WayPoints[i + 1]);
                distanceRestante = distanceBetweenPoints - (lengthWayPointLine - distanceParcourue);

                totalPoints.Add(WayPoints[i + 1]);
            }

            return totalPoints;
        }


        /// <summary>
        /// This function finds the trajectory segment closest to the robot by looking at the trajectory from the current trajectory segment to a certain curvilinear distance. 
        /// </summary>
        /// <param name="idx">Out parameter, index of the found closest trajectory segment.</param>
        /// <param name="MaxLookingDistance">Maximum looking distance in meters.</param>
        /// <returns>List of 2 Point2D containing as first element the start of the trajectory segment, and as second element the end of the trajectory segment.</returns>
        public List<Point2D> FindTrajectorySegment(Robot r, out int idx, int MaxLookingDistance = 20)
        {
            double minDist = 100000000;
            int minI = 0;
            for (int i = r.MinIdxTraj; i < (int)Math.Min(r.Traj.TrajectoryPoints.Count - 1, r.MinIdxTraj + MaxLookingDistance); i++)
            {
                Point2D P1 = r.Traj.TrajectoryPoints[i];
                Point2D P2 = r.Traj.TrajectoryPoints[i + 1];
                Point2D MidPoint = new Point2D((P1.X + P2.X) / 2, (P1.Y + P2.Y) / 2);
                double curDist = MidPoint.Norm(r.Position);
                if (curDist < minDist)
                {
                    minDist = curDist;
                    minI = i;
                }
            }
            List<Point2D> Segment = new List<Point2D>();
            Segment.Add(r.Traj.TrajectoryPoints[minI]);
            Segment.Add(r.Traj.TrajectoryPoints[minI + 1]);
            idx = minI;
            return Segment;
        }

    }
}
