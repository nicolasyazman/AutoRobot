using System;
using System.Collections.Generic;
using System.Text;

namespace AutoRobot
{
    public class Robot
    {
        // Current position of the center of gravity of the robot
        public Point2D Position { get; set; }

        // Current bearing of the vehicle w.r. to the X axis
        public double Bearing { get; set;  }

        // Steering angle between the front wheel and the body axis
        public double Psi { get; set; }

        public double Speed { get; set; }

        public double CurvilinearAbscissa { get; private set;}

        private Point2D RearAxlePosition { get; set; }
        private Point2D FrontAxlePosition { get; set; }

        public List<Point2D> Trajectory { get; set; }

        public double VehicleWidth { get; set; }

        public double VehicleLength { get; set; }

        public double VehicleHeight { get; set; }

        public double VehicleMass { get; set; }

        // DrivingForce
        public double Fd { get; set; }

        public int MinIdxTraj { get; set; }
        public Robot(Point2D pos, double initialBearing = 0)
        {
            this.Position = pos;
            this.VehicleLength = 1.5;
            this.VehicleWidth = 1.25;
            this.VehicleHeight = 0.2;
            this.VehicleMass = 3000;
            this.Speed = 0.1;
            this.Bearing = initialBearing;
            this.UpdateAxlesPositions();
            this.MinIdxTraj = 0;
        }

        public Point2D MoveRobot(Point2D newPosition)
        {
            this.Position = newPosition;



            // Do this command at the end
            this.UpdateAxlesPositions();
            return this.Position;
        }

        private void UpdateAxlesPositions()
        {
            double ContraryWiseAngle = Bearing + Math.PI;
            double HalfWidth = VehicleWidth / 2;

            double ThirdLength = VehicleLength / 3;
            RearAxlePosition = new Point2D(Position.X + Math.Cos(ContraryWiseAngle) * ThirdLength, Position.Y + Math.Sin(ContraryWiseAngle) * ThirdLength);
            FrontAxlePosition = new Point2D(Position.X + Math.Cos(Bearing) * ThirdLength, Position.Y + Math.Sin(Bearing));
        }


        // Calculate the distance between
        // point pt and the segment p1 --> p2.
        private double FindDistanceToSegment(
            Point2D pt, Point2D p1, Point2D p2, out Point2D closest)
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
        static double Delta_Bearing(double b1, double b2)
        {
            /*
			 * Optimal solution
			 *
			decimal d = 0;
 
			d = (b2-b1)%360;
 
			if(d>180)
				d -= 360;
			else if(d<-180)
				d += 360;
 
			return d;
			 *
			 * 
			 */


            //
            //
            //
            double d = 0;

            // Convert bearing to W.C.B
            if (b1 < 0)
                b1 += 360;
            if (b2 < 0)
                b2 += 360;

            ///Calculate delta bearing
            //and
            //Convert result value to Q.B.
            d = (b2 - b1) % 360;

            if (d > 180)
                d -= 360;
            else if (d < -180)
                d += 360;

            return d;

            //
            //
            //
        }

        // MaxWheelAngleRad = 0.3
        public Point2D MoveRobot(double ConsigneVitesse = 0.4, double MaxWheelAngleRad = 0.6)
        {
            if (MinIdxTraj > Trajectory.Count - 5)
                return new Point2D(0, 0);
            int closestIdx;
            List<Point2D> segment = FindTrajectorySegment(CurvilinearAbscissa, out closestIdx);
            MinIdxTraj = closestIdx;
            if (segment == null)
                return new Point2D(0, 0);
            double angle = segment[0].AbsoluteBearing( segment[1] );
            
            CurvilinearAbscissa += Math.Abs(Speed);

            double distanceTol = 0.1;
            Point2D closest; 
            double distanceToTrajSegment = FindDistanceToSegment(Position, segment[0], segment[1], out closest);

            if (distanceToTrajSegment < distanceTol)
            {
                Psi = Delta_Bearing(Bearing * 180 / Math.PI, angle * 180 / Math.PI);
                Psi *= Math.PI / 180;
                if (Psi < -MaxWheelAngleRad)
                    Psi = -MaxWheelAngleRad;
                if (Psi > MaxWheelAngleRad)
                    Psi = MaxWheelAngleRad;
                // Psi = (angle - Bearing);
            }
            else
            {
                double correctionAngle = 1000000;
                double correctionAngleTol = 0.3;
                double maxDeltaIdx = 20;
                double firstIdx = closestIdx;
                double minAngle = 100000;
                while (closestIdx - firstIdx < maxDeltaIdx && closestIdx < Trajectory.Count) //&& (correctionAngle > correctionAngleTol || 2 * Math.PI - correctionAngle > correctionAngleTol))
                {
                    correctionAngle = Position.AbsoluteBearing(Trajectory[closestIdx]);
                    if ((correctionAngle + Math.PI * 2) % Math.PI < minAngle)
                        minAngle = (correctionAngle + Math.PI*2) % (2*Math.PI);
                    closestIdx++;
                }
                Psi = Delta_Bearing(Bearing * 180 / Math.PI, minAngle * 180 / Math.PI);
                Psi *= Math.PI / 180;
                if (Psi < -MaxWheelAngleRad)
                   Psi = -MaxWheelAngleRad;
                if (Psi > MaxWheelAngleRad)
                    Psi = MaxWheelAngleRad;
                   
                //if (Psi > 0.3)
                 //   Psi = 0.3;
            }
            double deltax = 0, deltay = 0, deltatheta = 0, deltav = 0, deltaFd = 0, delpsi = 0;
            EstimateNextState(Speed, out deltax, out deltay, out deltatheta, out delpsi, out deltav, out deltaFd);

           /* while (deltatheta > 2 * Math.PI)
                deltatheta = deltatheta - 2 * Math.PI;
            while (deltatheta < 0)
                deltatheta += 2 * Math.PI;*/
            Bearing += deltatheta;

           // while (Bearing > 2 * Math.PI)
           //     Bearing = Bearing - 2 * Math.PI;
          //  while (Bearing < 0)
          //      Bearing = Bearing + 2 * Math.PI;
            Psi += delpsi;
            Speed += deltav;
            Fd += deltaFd;

            //Bearing = (Bearing + Math.PI * 2) % (Math.PI * 2);
            if (Speed > ConsigneVitesse)
                Speed = ConsigneVitesse;
            if (Fd > 50)
                Fd = 50;
            //if (Speed > 3)
            //    Speed = 3;
            Point2D nextPosition = CalculateRobotNextPositionCartesian(Position, deltax, deltay);
            MoveRobot(nextPosition);
            return Position;
        }

        private void EstimateNextState(double Speed, out double delx, out double dely, out double deltheta, out double delpsi, out double delv, out double delFd)
        {
            double x = Position.X;
            double y = Position.Y;
            double theta = Bearing;
            double vu = Speed;

            
            // Distance between the rear axle and the center of gravity
            double b = RearAxlePosition.Norm( Position);
            
            // l is the distance between the rear and front axles
            double l = RearAxlePosition.Norm(FrontAxlePosition);

            // Vehicle mass in g
            double m = VehicleMass;

            // Mass moment of inertia Note: Model of a cube
            double J = 1 / 12 * m * (Math.Pow(VehicleLength, 2) + Math.Pow(VehicleWidth, 2));
            // Setting basic Motor and load inertia in Nm/rad/s2
            //J = 0.00025;

            double gamma = Math.Pow(Math.Cos(Psi),2)*(Math.Pow(l,2)*m+(Math.Pow(b,2)*m+J)*Math.Pow(Math.Tan(Psi),2));

            
            // Terminal resistance of DC Motor in Ohm 
            double Ra = 0.5;

            // Terminal inductance of DC Motor in milliHenries
            double La = 1.5;


            double Tau_s = La / Ra;

            // ????
            double cs = 0;

            // Motor constant
            double MotorTorque = 10;
            double ResistivePowerLoss = 3;
            double Km = MotorTorque / Math.Sqrt(ResistivePowerLoss);

            // Also known as Kv, 
            double Kb = 0.05;

            // Friction in Nm/rad/s
            double Bm = 0.0001;

            // Number of teeth on the gears connecting the axles
            double Nw = 10;

            // Radius of wheel
            double Rw = 0.45;

            // Number of teeth on the gears connecting the motor
            double Nm = 10;

            // Input voltage in [-5,5]Volts
            double u1 = 2.8;
            double u2 = 2.8;
            
             delx = (Math.Cos(theta) - ((b * Math.Tan(Psi)) / l)  * Math.Sin(theta)) * vu;
             dely = (Math.Sin(theta) + ((b * Math.Tan(Psi)) / l) * Math.Cos(theta)) * vu;
             deltheta = (Math.Tan(Psi) / l) * vu;
             delpsi = 1 / Tau_s * Psi + cs * u2;
             delv = vu * (Math.Pow(b, 2) * m + J) * Math.Tan(Psi) / gamma * delpsi + Math.Pow(l, 2) * Math.Pow(Math.Cos(Psi), 2) / gamma * Fd;
             delFd = -Ra / La * Fd - ((Km * Kb + Ra * Bm) * Math.Pow(Nw,2)) / (La*Math.Pow(Nm,2)*Math.Pow(Rw,2)) * vu + (Km * Nw / (La * Nm * Rw) * u1);

        }

        public double CurvilinearAbscissaCumul(int endIndex)
        {
            double cumul = 0;
            if (endIndex >= Trajectory.Count)
                endIndex = Trajectory.Count-1;
            for (int i = 0; i < endIndex; i++)
            {
                cumul += Trajectory[i].Norm( Trajectory[i + 1]);
            }
            return cumul;
        }

        public List<Point2D> FindTrajectorySegment(double curvilinearAbsicca, out int idx)
        {
            double minDist = 100000000;
            int minI = 0;
            for (int i = this.MinIdxTraj; i < (int)Math.Min(Trajectory.Count-1, this.MinIdxTraj + 20); i++)
            {
                Point2D P1 = Trajectory[i];
                Point2D P2 = Trajectory[i + 1];
                Point2D MidPoint = new Point2D((P1.X + P2.X) / 2, (P1.Y + P2.Y) / 2);
                double curDist = MidPoint.Norm(Position);
                if (curDist < minDist)
                {
                    minDist = curDist;
                    minI = i;
                }
            }
            List<Point2D> Segment = new List<Point2D>();
            Segment.Add(Trajectory[minI]);
            Segment.Add(Trajectory[minI + 1]);
            idx = minI;
            return Segment;
            /*
            for (int i = 0; i < Trajectory.Count; i++)
            {
                if (CurvilinearAbscissaCumul(i) > curvilinearAbsicca)
                {
                    List<Point2D> currentSegment = new List<Point2D>();
                    currentSegment.Add(Trajectory[i-1]);
                    currentSegment.Add(Trajectory[i]);
                    return currentSegment;
                }
            }
            List<Point2D> currentSeg = new List<Point2D>();
            currentSeg.Add(Trajectory[0]);
            currentSeg.Add(Trajectory[1]);
            return currentSeg;
            */
        }

        public Point2D CalculateRobotNextPositionCartesian(Point2D origin, double xToMove, double yToMove)
        {
            return new Point2D(origin.X + xToMove, origin.Y + yToMove);          
        }

        public Point2D CalculateRobotNextPositionPolar(Point2D origin, double angle, double distance)
        {
            double xToMove = Math.Cos(angle) * distance;
            double yToMove = Math.Sin(angle) * distance;
            return CalculateRobotNextPositionCartesian(origin, xToMove, yToMove);
        }

        public List<Point2D> CalculateIntermediatePointsBetween2Points(Point2D origin, Point2D dest, double nbPoints)
        {

            double distance              = origin.Norm(dest);
            double distanceBetweenPoints = distance / (nbPoints + 1);
            double angle = origin.AbsoluteBearing( dest);

            List<Point2D> res = new List<Point2D>();
            for(int i=0; i<nbPoints; i++)
            {
                Point2D pointIntermediate = CalculateRobotNextPositionPolar(origin, angle, (distanceBetweenPoints * (i + 1)));
                res.Add(pointIntermediate);
            }

            return res;
        }

        public List<Point2D> CalculateIntermediatePointsSeparatedByDistance(Point2D origin, Point2D dest, double distanceBetweenPoints, double accumulatedDistance = 0)
        {
            double totalDistance = origin.Norm( dest);
            double numberPoints = (int)Math.Floor((totalDistance) / distanceBetweenPoints);
            double angle = origin.AbsoluteBearing( dest);

            List<Point2D> res = new List<Point2D>();
            int i = 0;
            while ((i) * distanceBetweenPoints + accumulatedDistance < totalDistance)
            {
                Point2D pointIntermediate = CalculateRobotNextPositionPolar(origin, angle, (distanceBetweenPoints * (i) + accumulatedDistance));
                res.Add(pointIntermediate);
                i++;
            }

            return res;
        }

        public List<Point2D> CalculateIntermediatePointsBetweenWayPoints(List<Point2D> WayPoints, double distanceBetweenPoints)
        {
            List<Point2D> totalPoints = new List<Point2D>();
            double distanceRestante = 0;
            for (int i = 0; i < WayPoints.Count-1; i++)
            {
                List<Point2D> currentPoints = CalculateIntermediatePointsSeparatedByDistance(WayPoints[i], WayPoints[i + 1], distanceBetweenPoints, distanceRestante);
                
                totalPoints.AddRange(currentPoints);
                double distanceParcourue = 0;
                for (int j = 0; j < currentPoints.Count - 1; j++)
                    distanceParcourue += currentPoints[j].Norm( currentPoints[j + 1]);

                distanceParcourue += distanceRestante;
                double lengthWayPointLine = WayPoints[i].Norm( WayPoints[i + 1]);
                distanceRestante = distanceBetweenPoints - (lengthWayPointLine - distanceParcourue);

                totalPoints.Add(WayPoints[i + 1]);
            }

            return totalPoints;
        }
    }
}
