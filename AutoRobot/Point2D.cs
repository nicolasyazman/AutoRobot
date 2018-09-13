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
            double aangle = Math.Atan2(target.Y - source.Y, target.X - source.X);
            double degangle = aangle * 180 / Math.PI;
            double res = (degangle + 360) % 360;
            return res * Math.PI / 180;
            //return aangle;
        }

        public static double Norm(Point2D point1, Point2D point2)
        {
            return Math.Sqrt(Math.Pow(point1.X-point2.X, 2) + Math.Pow(point1.Y-point2.Y, 2));
        }
    }
}
