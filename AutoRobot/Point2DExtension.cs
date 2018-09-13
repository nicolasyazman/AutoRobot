using AutoRobot;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AutoRobot
{
    internal static class Point2DExtension
    {
        public static double AbsoluteBearing( this Point2D @this, Point2D target)
        {
            double aangle = Math.Atan2(target.Y - @this.Y, target.X - @this.X);
            double degangle = aangle * 180 / Math.PI;
            double res = (degangle + 360) % 360;
            return res * Math.PI / 180;
        }

        public static double Norm(this Point2D @this, Point2D point2)
        {
            return Math.Sqrt(Math.Pow(@this.X - point2.X, 2) + Math.Pow(@this.Y - point2.Y, 2));
        }
    }
}
