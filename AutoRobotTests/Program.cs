using System;
using NUnit;

using AutoRobot;
using NUnit.Framework;
using FluentAssertions;

namespace AutoRobotTests
{

    [TestFixture]
    class Program
    {
        /*
        static void Main(string[] args)
        {
            Robot b;
            Console.WriteLine("Hello World!");
        }*/

        [TestCase]
        public static void TestBearingShouldBeWithinMinusPIPlusPI()
        {
            Random rand;
            double x1, y1, x2, y2;

            rand = new Random();
            x1 = rand.NextDouble();
            y1 = rand.NextDouble();
            x2 = rand.NextDouble();
            y2 = rand.NextDouble();

            Point2D orig = new Point2D(x1,y1);
            Point2D dest = new Point2D(x2,y2);
            double angle = Point2D.AbsoluteBearing(orig, dest);

            angle.Should().BeInRange(-Math.PI, Math.PI);
        }

        [TestCase]
        public static void TestBearingShouldMakeYouMoveToRightPosition()
        {
            Random rand;
            double x1, y1, x2, y2;

            rand = new Random();
            x1 = rand.NextDouble();
            y1 = rand.NextDouble();
            x2 = rand.NextDouble();
            y2 = rand.NextDouble();

            Point2D orig = new Point2D(x1, y1);
            Point2D dest = new Point2D(x2, y2);

            double angle = Point2D.AbsoluteBearing(orig, dest);

            Robot r = new Robot(orig);

            double dx = x2 - x1;
            double dy = y2 - y1;
            double distance = Math.Sqrt(dx*dx+dy*dy);
            r.MoveRobotPolar(angle, distance);

            r.Position.X.Should().BeApproximately(x2, 0.001);
            r.Position.Y.Should().BeApproximately(y2, 0.001);
        }
    }
}