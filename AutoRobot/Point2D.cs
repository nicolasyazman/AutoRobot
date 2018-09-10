using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AutoRobot
{
    public class Point2D
    {
        public double X { get; }
        public double Y { get; }

        public Point2D(double x, double y)
        {
            X = x;
            Y = y;
        }

        public static double AbsoluteBearing(Point2D source, Point2D target)
        {
            return Math.Atan2(target.Y - source.Y, target.X - source.X);
        }
    }
}
