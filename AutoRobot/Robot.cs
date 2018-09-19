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

        public double InputVoltage1 { get; set; }

        public double InputVoltage2 { get; set; }

        // DrivingForce
        public double Fd { get; set; }

        public int MinIdxTraj { get; set; }
        public Robot(Point2D pos, double initialBearing = 0)
        {
            this.Position = pos;
            this.VehicleLength = 1.5;
            this.VehicleWidth = 1.25;
            this.VehicleHeight = 0.2;
            this.VehicleMass = 5000;
            this.Speed = 0.1;
            this.Bearing = initialBearing;
            this.UpdateAxlesPositions();
            this.MinIdxTraj = 0;
            this.InputVoltage1 = 0;
            this.InputVoltage2 = 0;
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
        public Point2D MoveRobot(out double RegulatedVoltageU1, out double RegulatedVoltageU2, double ConsigneVitesse = 0.4, double MaxWheelAngleRad = 0.6, double K = 0.1, double inputVolu2 = 2.8)
        {
            ConsigneVitesse /= (1 + Math.Abs(this.InputVoltage2));

            double diff = ConsigneVitesse - Speed;
            double PsiConsigne = 0;

            double gain = diff * K;
            RegulatedVoltageU1 = this.InputVoltage1 + gain;
            RegulatedVoltageU2 = this.InputVoltage2;

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
                PsiConsigne = Delta_Bearing(Bearing * 180 / Math.PI, angle * 180 / Math.PI);
                PsiConsigne *= Math.PI / 180;
         /*       if (PsiConsigne < -MaxWheelAngleRad)
                    PsiConsigne = -MaxWheelAngleRad;
                if (PsiConsigne > MaxWheelAngleRad)
                    PsiConsigne = MaxWheelAngleRad;*/
                // Psi = (angle - Bearing);
            }
            else
            {
                double correctionAngle = 1000000;
                double maxDeltaIdx = 20;
                double firstIdx = closestIdx;
                double minAngle = 100000;
                closestIdx = 0;
                while (closestIdx - firstIdx < maxDeltaIdx && closestIdx < Trajectory.Count) //&& (correctionAngle > correctionAngleTol || 2 * Math.PI - correctionAngle > correctionAngleTol))
                {
                    correctionAngle = Position.AbsoluteBearing(Trajectory[closestIdx]);
                    if ((correctionAngle + Math.PI * 2) % Math.PI < minAngle)
                        minAngle = (correctionAngle + Math.PI*2) % (2*Math.PI);
                    closestIdx++;
                }
                PsiConsigne = Delta_Bearing(Bearing * 180 / Math.PI, minAngle * 180 / Math.PI);
                PsiConsigne *= Math.PI / 180;

               

                
         /*       if (PsiConsigne < -MaxWheelAngleRad)
                    PsiConsigne = -MaxWheelAngleRad;
                if (PsiConsigne > MaxWheelAngleRad)
                    PsiConsigne = MaxWheelAngleRad;
                   */
                //if (Psi > 0.3)
                 //   Psi = 0.3;
            }

            double diffPsi = Delta_Bearing(Psi * 180 / Math.PI, PsiConsigne * 180 / Math.PI) * Math.PI / 180;
            
            
            double gainPsi = diffPsi * 1.2;

            RegulatedVoltageU2 = RegulatedVoltageU2 + gainPsi;
            if (RegulatedVoltageU2 > 5)
                RegulatedVoltageU2 = 5;
            if (RegulatedVoltageU2 < -5)
                RegulatedVoltageU2 = -5;
            double deltax = 0, deltay = 0, deltatheta = 0, deltav = 0, deltaFd = 0, delpsi = 0;
            // EstimateNextState(Speed, out deltax, out deltay, out deltatheta, out delpsi, out deltav, out deltaFd, InputVoltage1, inputVolu2);
            EstimateNextState(Speed, out deltax, out deltay, out deltatheta, out delpsi, out deltav, out deltaFd, RegulatedVoltageU1, RegulatedVoltageU2);
            
            Bearing += deltatheta;

            Psi += delpsi;
            Speed += deltav;
            Fd += deltaFd;
            
            Point2D nextPosition = CalculateRobotNextPositionCartesian(Position, deltax, deltay);
            MoveRobot(nextPosition);
            this.InputVoltage1 = RegulatedVoltageU1;
            if (this.InputVoltage1 > 5)
                this.InputVoltage1 = 5;
            if (this.InputVoltage1 < -5)
                this.InputVoltage1 = -5;

            this.InputVoltage2 = RegulatedVoltageU2;
            if (this.InputVoltage2 > 5)
                this.InputVoltage2 = 5;
            if (this.InputVoltage2 < -5)
                this.InputVoltage2 = -5;

            Bearing = (Bearing + Math.PI * 2) % (Math.PI * 2);
            return Position;
        }

        private void EstimateNextState(double Speed, out double delx, out double dely, out double deltheta, out double delpsi, out double delv, out double delFd, double u1 = 2.8, double u2 = 2.8)
        {
            double x = Position.X;
            double y = Position.Y;
            double theta = Bearing;
            double vu = Speed;

            //u1 = 2.8;
            
            // Distance between the rear axle and the center of gravity
            double b = RearAxlePosition.Norm( Position);
            
            // l is the distance between the rear and front axles
            double l = RearAxlePosition.Norm(FrontAxlePosition);

            // Vehicle mass in g
            double m = VehicleMass;

            // Mass moment of inertia Note: Model of a cube
            double J = 1 / 12 * m * (Math.Pow(VehicleLength * 1000, 2) + Math.Pow(VehicleWidth * 1000, 2));
            // Setting basic Motor and load inertia in Nm/rad/s2
            //J = 0.00025;

            double gamma = Math.Pow(Math.Cos(Psi),2)*(Math.Pow(l,2)*m+(Math.Pow(b,2)*m+J)*Math.Pow(Math.Tan(Psi),2));

            
            // Terminal resistance of DC Motor in Ohm 
            double Ra = 0.5;

            // Terminal inductance of DC Motor in milliHenries
            double La = 1.5;


            double Tau_s = La / Ra;

            // ????
            // double cs = 0.6;
            double cs = 0.1;
            // Motor constant
            double MotorTorque = 10;
            double ResistivePowerLoss = 3;
            double Km = MotorTorque / Math.Sqrt(ResistivePowerLoss);

            // Also known as Kv, 
            double Kb = 0.05;

            // Friction in Nm/rad/s
            //double Bm = 0.0001; // Found on net
            double Bm = 0.001;
            // Number of teeth on the gears connecting the axles
            double Nw = 10;

            // Radius of wheel
            double Rw = 0.45;

            // Number of teeth on the gears connecting the motor
            double Nm = 10;

            // u1 and u2 are Input voltage in [-5,5]Volts
            
            
             delx = (Math.Cos(theta) - ((b * Math.Tan(Psi)) / l)  * Math.Sin(theta)) * vu;
             dely = (Math.Sin(theta) + ((b * Math.Tan(Psi)) / l) * Math.Cos(theta)) * vu;
             deltheta = (Math.Tan(Psi) / l) * vu;
             delpsi = 1 / Tau_s * Psi + cs * this.InputVoltage2;


            double MaxWheelAngleRad = 0.6;
            if (Psi + delpsi < -MaxWheelAngleRad)
            {
                delpsi = 0;
                Psi = -MaxWheelAngleRad;
            }
            if (Psi + delpsi > MaxWheelAngleRad)
            {
                delpsi = 0;
                Psi = MaxWheelAngleRad;
            }

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

        public List<Point2D> FindTrajectorySegment(double curvilinearAbsicca, out int idx, int MaxLookingDistance = 20)
        {
            double minDist = 100000000;
            int minI = 0;
            for (int i = this.MinIdxTraj; i < (int)Math.Min(Trajectory.Count-1, this.MinIdxTraj + MaxLookingDistance); i++)
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

        public List<Point2D> GetEstimatedFuturTrajectory(double estimatedDistance = 20)
        {
            List<Point2D> futurTrajectory = new List<Point2D>();
            Point2D actualPosition = new Point2D(this.Position.X, this.Position.Y);
            Robot estimatedRobot   = new Robot(actualPosition);
            estimatedRobot.Trajectory = this.Trajectory;
            estimatedRobot.Speed = this.Speed;
            estimatedRobot.Fd = 10;
            estimatedRobot.Bearing = this.Bearing;
            estimatedRobot.MinIdxTraj = this.MinIdxTraj;
            estimatedRobot.InputVoltage1 = 2.8;
            futurTrajectory.Add(actualPosition);
            double distanceCumulee = 0;

            while (distanceCumulee < estimatedDistance && estimatedRobot.MinIdxTraj < estimatedRobot.Trajectory.Count - 5)
            {
                double RegulatedVoltageU1;
                double RegulatedVoltageU2;
                estimatedRobot.MoveRobot(out RegulatedVoltageU1, out RegulatedVoltageU2, 0.6, 0.6, 1, this.InputVoltage2);
                futurTrajectory.Add(estimatedRobot.Position);
                distanceCumulee += futurTrajectory[futurTrajectory.Count -1].
                    Norm(futurTrajectory[futurTrajectory.Count - 2]);
            }

            return futurTrajectory;
        }

        public bool IsObstacleInTrajectory(List<Point2D> ObstaclesPos, double AnticipationDistance = 20)
        {
            for (int obsId = 0; obsId < ObstaclesPos.Count; obsId++)
            {
                List<Point2D> FutureRealTrajectory = GetEstimatedFuturTrajectory(AnticipationDistance);
                List<List<Point2D>> Corridor = GetVehicleCorridor(FutureRealTrajectory);
                for (int i = 0; i < Corridor.Count; i++)
                {
                    if (IsPointInsideRectangle(Corridor[i],ObstaclesPos[obsId]))
                    {
                        return true;
                    }
                }
            }
            return false;
        }


        public List<List<Point2D>> GetVehicleCorridor(List<Point2D> Trajectory)
        {
            List<List<Point2D>> corridor = new List<List<Point2D>>();

            int i;
            for(i = 0; i < Trajectory.Count-1; i++)
            {
                double angle = Trajectory[i].AbsoluteBearing(Trajectory[i + 1]);
                List<Point2D> vehicleBoundaries = new List<Point2D>();
                double Margin = 0.5;
                double distForward = this.VehicleLength + 0.5;
                double distBehind = this.VehicleLength + 0.5 ;

                double distLeft = (this.VehicleWidth / 2 + Margin);

                Point2D MidPointForward = new Point2D(Trajectory[i].X + Math.Cos(angle) * distForward, Trajectory[i].Y + Math.Sin(angle) * distForward);
                Point2D MidPointBehind = new Point2D(Trajectory[i].X + Math.Cos(angle + Math.PI) * distForward, Trajectory[i].Y + Math.Sin(angle + Math.PI) * distForward);

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

        bool PointInTriangle(Point2D pt, Point2D v1, Point2D v2, Point2D v3)
        {
            bool b1, b2, b3;

            b1 = sign(pt, v1, v2) < 0;
            b2 = sign(pt, v2, v3) < 0;
            b3 = sign(pt, v3, v1) < 0;

            return ((b1 == b2) && (b2 == b3));
        }

        public bool IsPointInsideRectangle(List<Point2D> rectPoints, Point2D point)
        {
            rectPoints.Sort((point1, point2) =>  point1.Y.CompareTo(point2.Y));
            if (PointInTriangle(point, rectPoints[0], rectPoints[1], rectPoints[2])
                || PointInTriangle(point, rectPoints[0], rectPoints[1], rectPoints[2]))
                return true;
            return false;
        }

        
        public int[,] CreatePotentialField(List<Point2D> ObstaclesGC, double ObstacleSize, out int VehGridX, out int VehGridY, double GridWidth = 100, double GridHeight = 100, double SafetyDistance = 10, double LookIdx = 10)
        {
            double GridResolution = 0.1;

            int NumberOfCellsPerRow = (int)Math.Ceiling(GridWidth / GridResolution);
            int NumberOfRows = (int)Math.Ceiling(GridHeight / GridResolution);

            int[,] Grid = new int[NumberOfRows, NumberOfCellsPerRow]; 
        /*    for (int y = 0; y < NumberOfRows; y++)
            {
                for (int x = 0; x < NumberOfCellsPerRow; x++)
                {
                    Grid[y, x] = 0;
                }
            }
            */
            double GridStartX = NumberOfCellsPerRow / 2;
            double GridStartY = NumberOfRows / 2;

            double LookForwardDistanceMeters = 5;
            int LookForward = 10;
            double CurvAbsCumul = 0;

            while (CurvAbsCumul < LookForwardDistanceMeters && MinIdxTraj + LookForward < Trajectory.Count - 1)
            {
                CurvAbsCumul += Trajectory[MinIdxTraj + LookForward].Norm(Trajectory[MinIdxTraj + LookForward + 1]);
                LookForward++;
            }
            int minidx;
            FindTrajectorySegment(0, out minidx, 100000);
            for (int i = (int)(Math.Min(minidx + LookForward, Trajectory.Count)); i < (int)(Math.Min(Trajectory.Count,MinIdxTraj+LookForward+LookIdx)); i++)
            {
                Point2D trajPoint = Trajectory[i];
                int Gridx = (int)(trajPoint.X / GridResolution + GridStartX);
                int Gridy = (int)(trajPoint.Y / GridResolution + GridStartY);
                if (Gridx >= 0 && Gridx < NumberOfCellsPerRow && Gridy >= 0 && Gridy < NumberOfRows)
                    Grid[Gridy, Gridx] = -3;
            }
            for (int i = 0; i < ObstaclesGC.Count; i++)
            {
                double obstacleX = ObstaclesGC[i].X / GridResolution + GridStartX;
                double obstacleY = ObstaclesGC[i].Y / GridResolution + GridStartY;
                int StartX = Math.Max(0,(int)(obstacleX - (ObstacleSize / GridResolution)));
                int EndX = Math.Min(NumberOfCellsPerRow - 1, (int)(obstacleX + (ObstacleSize / GridResolution)));
                int StartY = Math.Max(0, (int)(obstacleY - ObstacleSize / GridResolution));
                int EndY = Math.Min(NumberOfRows - 1,(int)(obstacleY + ObstacleSize / GridResolution));
                for (int y = StartY; y < EndY; y++)
                {
                    for (int x = StartX; x < EndX; x++)
                    {
                        if (new Point2D(x,y).Norm(new Point2D(obstacleX,obstacleY)) < (SafetyDistance / GridResolution))
                        {
                            Grid[y, x] = -1;
                        }
                    }
                }
            }

            VehGridX = (int)Math.Floor(Position.X / GridResolution + GridStartX);
            VehGridY = (int)Math.Floor(Position.Y / GridResolution + GridStartY);
            //Grid[VehGridY, VehGridX] = 2;
            return Grid;
        }


        public List<Point2D> FindShortestPath(int[,] Grid, int GridWidth, int GridHeight, int StartX, int StartY)
        {
            double GridResolution = 0.1;
            GridWidth = (int)(GridWidth / GridResolution);
            GridHeight = (int)(GridHeight / GridResolution);


            double GridStartX = GridWidth / 2;
            double GridStartY = GridHeight / 2;
            Queue<Tuple<int, int, int>> queue = new Queue<Tuple<int, int, int>>();
          
            queue.Enqueue(new Tuple<int, int, int>(StartX, StartY, 0));
            int X = 0, Y = 0;
            int Val = 0;
            int MaximumVal = 0;
            while (!(queue.Count == 0) && Val > -2)
            {
                Tuple<int,int, int> Position = queue.Dequeue();
                
                X = Position.Item1;
                Y = Position.Item2;                
                Val = Position.Item3;
                if (Val > MaximumVal)
                    MaximumVal = Val;
                if (Val > -1) // Destination not found
                {
                    // West
                    if (X - 1 >= 0 && Grid[Y, X - 1] != -1)
                    {
                        if (Grid[Y, X - 1] == 0)
                        {
                            Grid[Y, X - 1] = Val + 1;
                            queue.Enqueue(new Tuple<int, int, int>(X - 1, Y, Val + 1));
                        }
                        else if (Grid[Y, X - 1] < -1)
                        {
                            queue.Enqueue(new Tuple<int, int, int>(X - 1, Y, -2));
                        }
                        
                    }
                    // North West
                    if (X - 1 >= 0 && Y - 1 >= 0 && Grid[Y - 1, X - 1] != -1)
                    {
                        if (Grid[Y - 1, X - 1] == 0)
                        {
                            Grid[Y - 1, X - 1] = Val + 1;
                            queue.Enqueue(new Tuple<int, int, int>(X - 1, Y - 1, Val + 1));
                        }
                        else if (Grid[Y - 1, X - 1] < -1)
                        {
                            queue.Enqueue(new Tuple<int, int, int>(X - 1, Y - 1, -2));
                        }
                    }
                    // East
                    if (X + 1 < GridWidth && Grid[Y, X + 1] != -1)
                    {
                        if (Grid[Y, X + 1] == 0)
                        {
                            Grid[Y, X + 1] = Val + 1;
                            queue.Enqueue(new Tuple<int, int, int>(X + 1, Y, Val + 1));
                        }
                        else if (Grid[Y,X+1] < -1)
                        {
                            queue.Enqueue(new Tuple<int, int, int>(X + 1, Y, -2));
                        }
                    }
                    // North East
                    if (X + 1 < GridWidth  && Y - 1 >= 0 && Grid[Y - 1, X + 1] != -1)
                    {
                        if (Grid[Y - 1, X + 1] == 0)
                        {
                            Grid[Y - 1, X + 1] = Val + 1;
                            queue.Enqueue(new Tuple<int, int, int>(X + 1, Y - 1, Val + 1));
                        }
                        else if (Grid[Y - 1, X + 1] < -1)
                        {
                            queue.Enqueue(new Tuple<int, int, int>(X + 1, Y - 1, -2));
                        }
                    }

                    // North
                    if (Y - 1 >= 0 && Grid[Y-1, X] != -1)
                    {
                        if (Grid[Y - 1, X] == 0)
                        {
                            Grid[Y - 1, X] = Val + 1;
                            queue.Enqueue(new Tuple<int, int, int>(X, Y - 1, Val + 1));
                        }
                        else if (Grid[Y-1,X] < -1)
                        {
                            queue.Enqueue(new Tuple<int, int, int>(X, Y - 1, -2));
                        }
                    }

                    // South
                    if (Y + 1 < GridHeight && Grid[Y+1, X] != -1)
                    {
                        if (Grid[Y + 1, X] == 0)
                        {
                            Grid[Y + 1, X] = Val + 1;
                            queue.Enqueue(new Tuple<int, int, int>(X, Y + 1, Val + 1));
                        }
                        else if (Grid[Y+1,X] < -1)
                        {
                            queue.Enqueue(new Tuple<int, int, int>(X, Y + 1, -2));
                        }
                    }
                    
                    // South West
                    if (Y + 1 < GridHeight && X - 1 >= 0 && Grid[Y + 1, X - 1] != -1)
                    {
                        if (Grid[Y + 1, X - 1] == 0)
                        {
                            Grid[Y + 1, X - 1] = Val + 1;
                            queue.Enqueue(new Tuple<int, int, int>(X - 1, Y + 1, Val + 1));
                        }
                        else if (Grid[Y + 1, X - 1] < -1)
                        {
                            queue.Enqueue(new Tuple<int, int, int>(X - 1, Y + 1, -2));
                        }
                    }
                    // South East
                    if (Y + 1 < GridHeight && X + 1 < GridWidth && Grid[Y + 1, X + 1] != -1)
                    {
                        if (Grid[Y + 1, X + 1] == 0)
                        {
                            Grid[Y + 1, X + 1] = Val + 1;
                            queue.Enqueue(new Tuple<int, int, int>(X + 1, Y + 1, Val + 1));
                        }
                        else if (Grid[Y + 1, X + 1] < -1)
                        {
                            queue.Enqueue(new Tuple<int, int, int>(X + 1, Y + 1, -2));
                        }
                    }
                }
                else if (Val < -1)
                {
                    Grid[Y, X] = MaximumVal + 1;
                }
            }

            if (Val != -2) // Not found objective
            {
                return null;
            }

            Val = Grid[Y, X];
            List<Point2D> ShortestPath = new List<Point2D>();
            //ShortestPath.Add(new Point2D(X, Y));
            int ValShouldBe = Val;
            // Now do the inverse way
            while (ValShouldBe > 0)
            {
                // West
                if (X - 1 >= 0 && Grid[Y, X - 1] == ValShouldBe)
                {
                    X = X - 1;
                    ShortestPath.Add(new Point2D((X - GridStartX )* GridResolution , (Y - GridStartY) * GridResolution));
                }
                // East
                else if (X + 1 < GridWidth && Grid[Y, X + 1] == ValShouldBe)
                {
                    X = X + 1;
                    ShortestPath.Add(new Point2D((X - GridStartX) * GridResolution, (Y - GridStartY) * GridResolution));
                }
                // North
                else if (Y - 1 >= 0 && Grid[Y - 1, X] == ValShouldBe)
                {

                    Y = Y - 1;
                    ShortestPath.Add(new Point2D((X - GridStartX) * GridResolution, (Y - GridStartY) * GridResolution));
                }
                // South
                else if (Y + 1 < GridHeight && Grid[Y + 1, X] == ValShouldBe)
                {
                    Y = Y + 1;
                    ShortestPath.Add(new Point2D((X - GridStartX) * GridResolution, (Y - GridStartY) * GridResolution));
                }
                // South West
                else if (Y + 1 < GridHeight && X - 1 >= 0 && Grid[Y + 1, X - 1] == ValShouldBe)
                {
                    Y = Y + 1;
                    X = X - 1;
                    ShortestPath.Add(new Point2D((X - GridStartX) * GridResolution, (Y - GridStartY) * GridResolution));

                }
                // South East
                else if (Y + 1 < GridHeight && X + 1 < GridWidth && Grid[Y + 1, X + 1] == ValShouldBe)
                {
                    Y = Y + 1;
                    X = X + 1;
                    ShortestPath.Add(new Point2D((X - GridStartX) * GridResolution, (Y - GridStartY) * GridResolution));
                }

                // North West
                else if (Y - 1 >= 0 && X - 1 >= 0 && Grid[Y - 1, X - 1] == ValShouldBe)
                {

                    Y = Y - 1;
                    X = X - 1;
                    ShortestPath.Add(new Point2D((X - GridStartX) * GridResolution, (Y - GridStartY) * GridResolution));
                }

                // North East
                else if (Y - 1 >= 0 && X + 1 >= 0 && Grid[Y - 1, X + 1] == ValShouldBe)
                {

                    Y = Y - 1;
                    X = X + 1;
                    ShortestPath.Add(new Point2D((X - GridStartX) * GridResolution, (Y - GridStartY) * GridResolution));
                }
                ValShouldBe--;
            }
            ShortestPath.Reverse();
            
            return ShortestPath;
        }
    }
}
