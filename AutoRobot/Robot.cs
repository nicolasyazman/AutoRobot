using System;
using System.Collections.Generic;
using System.Text;

namespace AutoRobot
{
    public class Robot
    {
        public Point2D Position { get; set; }
        public double Bearing { get; }


        public Robot(Point2D pos)
        {
            this.Position = pos;
        }

        public Point2D MoveRobotCartesian(double xToMove, double yToMove)
        {
            this.Position = new Point2D(Position.X + xToMove, Position.Y + yToMove);
            return this.Position;
        }

        public Point2D MoveRobotPolar(double angle, double distance)
        {
            double xToMove = Math.Cos(angle) * distance;
            double yToMove = Math.Sin(angle) * distance;
            return MoveRobotCartesian(xToMove, yToMove);
        }

        public List<Point2D> CalculateIntermediatePointsBetween2Points(Point2D origin, Point2D dest, double nbPoints)
        {
            double dx = dest.X - origin.X;
            double dy = dest.Y - origin.Y;

            double distance              = Math.Sqrt(dx * dx + dy * dy);
            double distanceBetweenPoints = distance / (nbPoints + 1);
            double angle = Point2D.AbsoluteBearing(origin, dest);

            List<Point2D> res = new List<Point2D>();
            for(int i=0; i<nbPoints; i++)
            {
                Point2D pointIntermediate = new Point2D(
                    origin.X + Math.Cos(angle) * (distanceBetweenPoints * (i+1)),
                    origin.Y + Math.Sin(angle) * (distanceBetweenPoints * (i + 1))
                );

                res.Add(pointIntermediate);
            }

            return res;
        }
    }
}
