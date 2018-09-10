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
    }
}
