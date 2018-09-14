using System;
using NUnit;
using AutoRobotExercises;
using AutoRobot;
using NUnit.Framework;
using FluentAssertions;
using System.Collections.Generic;

namespace AutoRobotTests
{

    [TestFixture]
    class Program
    {

        [TestCase]
        [Repeat(300)]
        public static void TestBearingShouldBeWithinMinusPIPlusPI()
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
            double angle = orig.AbsoluteBearing(dest);

            angle.Should().BeInRange(0, 2 * Math.PI);
        }

        [TestCase]
        [Repeat(300)]
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

            double angle = orig.AbsoluteBearing(dest);

            MyRobot r = new MyRobot(orig, angle);

            double distance = orig.Norm(dest);
            r.MoveRobot(r.CalculateRobotNextPositionPolar(orig, angle, distance));

            r.Position.X.Should().BeApproximately(x2, 0.001);
            r.Position.Y.Should().BeApproximately(y2, 0.001);
        }

        //prend un point d origine et de destination et un nombre de pts intermediaires et calcul la posi
        //position de ces points sur la ligne

        [TestCase]
        [Repeat(300)]
        public static void TestCalculIntermediatePoint()
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

            double angle = orig.AbsoluteBearing(dest);
            Robot r = new Robot(orig);

            int nbPoints = rand.Next(3, 10);
            List<Point2D> intermediatePoints = r.CalculateIntermediatePointsBetween2Points(orig, dest, nbPoints);

            intermediatePoints.Should().NotBeNullOrEmpty();
            intermediatePoints.Count.Should().Be(nbPoints);
            intermediatePoints.Should().NotContainNulls();
            intermediatePoints.Should().NotContain(orig);
            intermediatePoints.Should().NotContain(dest);

            double distanceTot = orig.Norm(dest);

            for (int i = 0; i < nbPoints; i++)
            {
                orig.AbsoluteBearing(intermediatePoints[i]).Should().BeApproximately(angle, 0.001);
            }

            double distCumul = 0;
            intermediatePoints.Insert(0, orig);
            intermediatePoints.Add(dest);
            for (int i = 0; i < nbPoints + 1; i++)
            {
                double distance = intermediatePoints[i].Norm(intermediatePoints[i + 1]);
                distCumul += distance;
            }
            distCumul.Should().BeApproximately(distanceTot, 0.0001);

            // Check equidistance of points
            for (int i = 0; i < nbPoints; i++)
            {
                intermediatePoints[i].Norm(intermediatePoints[i + 1]).Should().BeApproximately(intermediatePoints[i + 1].Norm(intermediatePoints[i + 2]), 0.0001);
            }
        }


        [TestCase]
        [Repeat(300)]
        public static void TestCalculIntermediatePointsWayPoints()
        {
            Random rand;
            double x, y;


            rand = new Random();

            int numberWayPoints = rand.Next(5, 10);

            List<Point2D> WayPoints = new List<Point2D>();
            for (int i = 0; i < numberWayPoints; i++)
            {
                x = rand.NextDouble() * 10000;
                y = rand.NextDouble() * 10000;
                Point2D wayPoint = new Point2D(x, y);
                WayPoints.Add(wayPoint);
            }


            Robot r = new Robot(WayPoints[0]);

            double distanceBetweenPoints = rand.NextDouble() * 100 + 0.05;
            List<Point2D> intermediatePoints = r.CalculateIntermediatePointsBetweenWayPoints(WayPoints, distanceBetweenPoints);

            intermediatePoints.Should().NotBeNullOrEmpty();
            intermediatePoints.Should().NotContainNulls();
            intermediatePoints.Count.Should().BeGreaterThan(WayPoints.Count);

            double totDistanceInterpPoints = 0;

            double totDistanceWayPoints = 0;

            for (int i = 0; i < WayPoints.Count - 1; i++)
                totDistanceWayPoints += WayPoints[i].Norm(WayPoints[i + 1]);

            double calculatedBetweenPoints1 = 0, calculatedBetweenPoints2 = 0;

            Point2D LastWaypoint = WayPoints[0];
            int intermediatePointNext = 0;
            for (int i = 1; i < intermediatePoints.Count - 1; i++)
            {
                calculatedBetweenPoints1 = intermediatePoints[i - 1].Norm(intermediatePoints[i]);
                calculatedBetweenPoints2 = intermediatePoints[i].Norm(intermediatePoints[i + 1]);

                if (WayPoints.Contains(intermediatePoints[i]))
                {
                    calculatedBetweenPoints1 += calculatedBetweenPoints2;
                    i++;
                    LastWaypoint = intermediatePoints[i];
                    intermediatePointNext++;
                }
                else
                {
                    LastWaypoint.AbsoluteBearing(intermediatePoints[i]).Should().BeApproximately(LastWaypoint.AbsoluteBearing(WayPoints[intermediatePointNext + 1]), 0.001);
                }
                calculatedBetweenPoints1.Should().BeInRange(distanceBetweenPoints - 0.0001, distanceBetweenPoints + 0.0001);


                totDistanceInterpPoints += calculatedBetweenPoints1;
            }
            totDistanceInterpPoints += calculatedBetweenPoints2;
            totDistanceInterpPoints.Should().BeApproximately(totDistanceWayPoints, 0.01);


        }

        [TestCase]
        public static void Test_robot_should_follow_trajectory()
        {
            Random rand;
            double x, y;


            rand = new Random();

            int numberWayPoints = rand.Next(5, 10);

            List<Point2D> WayPoints = new List<Point2D>();
            for (int i = 0; i < numberWayPoints; i++)
            {
                x = rand.NextDouble() * 10000;
                y = rand.NextDouble() * 10000;
                Point2D wayPoint = new Point2D(x, y);
                WayPoints.Add(wayPoint);
            }

            MyRobot robot = new MyRobot(WayPoints[0], 0);

            List<Point2D> trajectory = robot.CalculateIntermediatePointsBetweenWayPoints(WayPoints, 1.55);
            robot.Trajectory = WayPoints;
            robot.Speed = 1;

            for (int i = 0; i < 100; i++)
            {
                robot.MoveRobot(new Point2D(robot.Position.X +1 , robot.Position.Y +1));
            }
        }

        [TestCase]
        public static void Test_robot_acceleration_inferior_1g()
        {
            Random rand;
            double x, y;


            rand = new Random();

            int numberWayPoints = rand.Next(5, 10);

            List<Point2D> WayPoints = new List<Point2D>();
            for (int i = 0; i < numberWayPoints; i++)
            {
                x = rand.NextDouble() * 10000;
                y = rand.NextDouble() * 10000;
                Point2D wayPoint = new Point2D(x, y);
                WayPoints.Add(wayPoint);
            }

            MyRobot robot = new MyRobot(WayPoints[0], WayPoints[0].AbsoluteBearing(WayPoints[1]));

            List<Point2D> trajectory = robot.CalculateIntermediatePointsBetweenWayPoints(WayPoints, 1.55);
            robot.Trajectory = WayPoints;
            robot.Speed = 0;
            double lastSpeed = 0;

            for (int i = 0; i < 100; i++)
            {
                robot.MoveRobot(new Point2D(robot.Position.X + 100, robot.Position.Y));
                (robot.Speed - lastSpeed).Should().BeLessThan(9.81);
                lastSpeed = robot.Speed;
            }
        }
        [TestCase]
        public static void TestIsPointInsideRectangleFalse()
        {
            Point2D T1, T2, T3, P;
            P = new Point2D(0, 0);
            T1 = new Point2D(1, 1);
            T2 = new Point2D(2, 2);
            T3 = new Point2D(3, 3);
            List<Point2D> rect = new List<Point2D>();
            rect.Add(T1);
            rect.Add(T2);
            rect.Add(T3);
            Robot robot = new Robot(new Point2D(0, 0));

            robot.IsPointInsideRectangle(rect, P).Should().Equals(0);
        }

        [TestCase]
        public static void TestIsPointInsideRectangleTrue()
        {
            Point2D T1, T2, T3, P;
            P = new Point2D(1, 1);
            T1 = new Point2D(0, 0);
            T2 = new Point2D(3, 0);
            T3 = new Point2D(0, 3);
            List<Point2D> rect = new List<Point2D>();
            rect.Add(T1);
            rect.Add(T2);
            rect.Add(T3);
            Robot robot = new Robot(new Point2D(0, 0));

            robot.IsPointInsideRectangle(rect, P).Should().Equals(1);
        }
    }
}