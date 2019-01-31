using System;
using System.Collections.Generic;
using System.Text;

namespace AutoRobot
{
    public class Robot
    {

        /// <summary>
        /// Current position of the center of gravity of the robot.
        /// </summary>
        public Point2D Position { get; set; }


        /// <summary>
        /// Current bearing (yaw angle / heading) of the robot w.r. to the X axis.
        /// </summary>
        public double Bearing { get; set;  }

        /// <summary>
        /// Steering angle between the front wheel and the body axis.
        /// </summary>
        public double Psi { get; set; }

        /// <summary>
        /// Speed of the robot in meters/seconds.
        /// </summary>
        public double Speed { get; set; }


        public double CurvilinearAbscissa { get; set;}

        public PathPlanning PathPlanningModule { get; }

        public ObstacleAvoidanceModule ObstacleAvoidModule { get; }
 
        /// <summary>
        /// Position of the center of the rear axle of the robot, in meters.
        /// </summary>
        private Point2D RearAxlePosition { get; set; }

        /// <summary>
        /// Position of the center of the front axle of the robot, in meters.
        /// </summary>
        private Point2D FrontAxlePosition { get; set; }

        /// <summary>
        /// Trajectory the robot should follow.
        /// </summary>
        public Trajectory Traj { get; set; }

        /// <summary>
        /// Width of the robot in meters.
        /// </summary>
        public double VehicleWidth { get; set; }


        /// <summary>
        /// Length of the robot in meters.
        /// </summary>
        public double VehicleLength { get; set; }

        /// <summary>
        /// Height of the robot in meters.
        /// </summary>
        public double VehicleHeight { get; set; }

        /// <summary>
        /// Mass of the robot in kilograms.
        /// </summary>
        public double VehicleMass { get; set; }

        /// <summary>
        /// Value corresponding to the speed instruction of the robot. Should be within [-5,+5].
        /// </summary>
        public double InputVoltage1 { get; set; }

        /// <summary>
        /// Value corresponding to the yaw instruction of the robot. Should be within [-5,+5].
        /// </summary>
        public double SteeringVoltage { get; set; }
        
        /// <summary>
        /// Driving force. Not used.
        /// </summary>
        public double Fd { get; set; }


        /// <summary>
        /// Current index of the trajectory segment the robot is closest to.
        /// </summary>
        public int MinIdxTraj { get; set; }

        /// <summary>
        /// Constructor of the robot. Takes a position of origin and an initial heading.
        /// </summary>
        /// <param name="pos">Position of the robot in meters.</param>
        /// <param name="initialBearing">Heading of the robot in radians.</param>
        public Robot(Point2D pos, double initialBearing = 0)
        {
            this.Position = pos;
            this.VehicleLength = 0.5;
            this.VehicleWidth = 0.25;
            this.VehicleHeight = 0.2;
            this.VehicleMass = 1.45;
            this.Speed = 0.0;
            this.Bearing = initialBearing;
            this.UpdateAxlesPositions();
            this.MinIdxTraj = 0;
            this.InputVoltage1 = 0;
            this.SteeringVoltage = 0;
            this.Traj = new Trajectory(new List<Point2D>());
            this.PathPlanningModule = new PathPlanning(this);
            this.ObstacleAvoidModule = new ObstacleAvoidanceModule(this);
        }

        /// <summary>
        /// Not used.
        /// </summary>
        /// <param name="newPosition"></param>
        /// <returns></returns>
        public Point2D MoveRobot(Point2D newPosition)
        {
            this.Position = newPosition;
            // Do this command at the end
            this.UpdateAxlesPositions();
            return this.Position;
        }

        /// <summary>
        /// This recomputes and reassigns the position of the rear and front axles of the robot.
        /// </summary>
        private void UpdateAxlesPositions()
        {
            double ContraryWiseAngle = Bearing + Math.PI;
            double HalfWidth = VehicleWidth / 2;

            double ThirdLength = VehicleLength / 3;
            RearAxlePosition = new Point2D(Position.X + Math.Cos(ContraryWiseAngle) * ThirdLength, Position.Y + Math.Sin(ContraryWiseAngle) * ThirdLength);
            FrontAxlePosition = new Point2D(Position.X + Math.Cos(Bearing) * ThirdLength, Position.Y + Math.Sin(Bearing));
        }

        

        
        /// <summary>
        /// Moves the robot using the robot's internal parameters.
        /// </summary>
        /// <param name="RegulatedSpeedVoltage">Output parameter. Value representing how fast the robot should drive.</param>
        /// <param name="RegulatedSteeringVoltage">Output parameter. Value representing how hard the robot should turn.</param>
        /// <param name="ConsigneVitesse">Speed instruction. The robot should do its best to reach this speed.</param>
        /// <param name="MaxWheelAngleRad">Max angle the wheels can turn.</param>
        /// <param name="K">How hard the vehicle speeds up and turns.</param>
        /// <returns>Position of the robot after it has moved.</returns>
        public Point2D MoveRobot(out double RegulatedSpeedVoltage, out double RegulatedSteeringVoltage, double ConsigneVitesse = 0.4, double MaxWheelAngleRad = 0.6, double K = 0.1)
        {
            ConsigneVitesse /= (1 + Math.Abs(this.SteeringVoltage));

            double diff = ConsigneVitesse - Speed;
            double PsiConsigne = 0;

            double gain = diff * K;
            RegulatedSpeedVoltage = this.InputVoltage1 + gain;
            RegulatedSteeringVoltage = this.SteeringVoltage;
            

            PsiConsigne = PathPlanningModule.ComputeSteeringToFollowTrajectory();

            double diffPsi = Geometry.Delta_Bearing(Psi * 180 / Math.PI, PsiConsigne * 180 / Math.PI) * Math.PI / 180;
            
            
            double gainPsi = diffPsi * 0.1;

            RegulatedSteeringVoltage = RegulatedSteeringVoltage + gainPsi;
            if (RegulatedSteeringVoltage > 5)
                RegulatedSteeringVoltage = 5;
            if (RegulatedSteeringVoltage < -5)
                RegulatedSteeringVoltage = -5;
            double deltax = 0, deltay = 0, deltatheta = 0, deltav = 0, deltaFd = 0, delpsi = 0;

            EstimateNextState(Speed, out deltax, out deltay, out deltatheta, out delpsi, out deltav, out deltaFd, RegulatedSpeedVoltage, RegulatedSteeringVoltage);
            
            Bearing += deltatheta;

            Psi += delpsi;
            Speed += deltav;
            Fd += deltaFd;
            
            Point2D nextPosition = CalculateRobotNextPositionCartesian(Position, deltax, deltay);
            MoveRobot(nextPosition);
            this.InputVoltage1 = RegulatedSpeedVoltage;
            if (this.InputVoltage1 > 5)
                this.InputVoltage1 = 5;
            if (this.InputVoltage1 < -5)
                this.InputVoltage1 = -5;

            this.SteeringVoltage = RegulatedSteeringVoltage;
            if (this.SteeringVoltage > 5)
                this.SteeringVoltage = 5;
            if (this.SteeringVoltage < -5)
                this.SteeringVoltage = -5;

            Bearing = (Bearing + Math.PI * 2) % (Math.PI * 2);
            return Position;
        }

        /// <summary>
        /// Estimates the difference between the state variables of the robot at the moment t and t+1.
        /// </summary>
        /// <param name="Speed">Speed of the robot.</param>
        /// <param name="delx">Output parameter. Delta in X position. In meters.</param>
        /// <param name="dely">Output parameter. Delta in Y position. In meters.</param>
        /// <param name="deltheta">Output parameter. Delta in bearing. In radians.</param>
        /// <param name="delpsi">Output parameter. Delta in wheel angle. In radians.</param>
        /// <param name="delv">Output parameter. Delta in speed. In meters per second.</param>
        /// <param name="delFd">Output parameter. Delta in driving force. In newtons.</param>
        /// <param name="u1">Regulated speed instruction.</param>
        /// <param name="u2">Regulated wheel angle instruction.</param>
        private void EstimateNextState(double Speed, out double delx, out double dely, out double deltheta, out double delpsi, out double delv, out double delFd, double u1 = 2.8, double u2 = 2.8)
        {
            double Lf = Position.Norm(FrontAxlePosition);
            double Lr = Position.Norm(RearAxlePosition);

            double accel;

            accel = (u1 - this.InputVoltage1) / 15;
            Psi = u2;

            //Psi = this.SteeringVoltage / 100;
            // Car kinematics model
            double Beta = Math.Atan(Lr / (Lf + Lr) * Math.Tan(Psi));
            double kin_x = Speed * Math.Cos(Beta + Bearing);
            double kin_y = Speed * Math.Sin(Beta + Bearing);
            double kin_psi = Speed / Lr * Math.Sin(Beta);
            double kin_v = accel;
            

            // Car dynamics model

            double vx = Math.Cos(Beta) * Speed;
            double vy = Math.Sin(Beta) * Speed;
            double m = this.VehicleMass;

            // Yaw inertia:
            double Iz = 100;

            // Radius of wheel:
            double r = 35e-3;

            // Cornering stiffness coefficient front
            double Caf = 0;

            // Front wheel slip angle
            double af;
            if (vx > 0)
            {
                af = (vy + Lf * r) / vx - Psi;
            }
            else
            {
                af = 0;
            }
            // af = -(u2 - this.SteeringVoltage);
            // Using simplified Pajecka model (http://eprints.hud.ac.uk/id/eprint/1190/1/fulltext1.pdf)

            // Cornering stiffness coefficient rear


            // Belt compression modulus. The suggested value is 27e6 N / M²
            double E = 27*10e6;

            // Material thickness of the belt. It is suggested that a value of 0.015 m be used for road tyres and a value of 0.01 m for race tyres
            double b = 0.015;


            double w = r / 100;

            // Radius of the wheel
            double r_paj = r;
            double a = 1;

            // Parameter s can always be assumed to be 0.15 for road tyres and 0.1 for racing tyres
            double s = 0.15;

            double L =  2 * (r + w * a) * Math.Sin(Math.Acos(1 - (s * w * a / (r_paj + w * a))));
            double x = L / 6;
            double Car = 4 *E*b*Math.Pow(w,3) / (3*x*(2*Math.PI*(r_paj+w*a)-L));

            // Rear wheel slip angle
            double ar;
            if (vx > 0.1)
            {
                ar = (vy - Lr * r) / vx;
            }
            else
            {
                ar = 0;
            }
            ar = 0;
            Caf = Car;

            double Fcf = -Caf * af;
            double Fcr = -Car * ar;
            //Fcr = 0;
            // Longitudinal speed in the body frame
            double dyn_x = kin_psi * kin_y + vx;

            // Lateral speed in the body frame
            double dyn_y = -kin_psi * kin_x + 2 / VehicleMass * (Fcf * Math.Cos(Psi) + Fcr);

            // Yaw rate
            double dyn_psi = 2 / Iz * (Lf * Fcf - Lr * Fcr);

            // 
            double dyn_posx = kin_x * Math.Cos(Bearing) - kin_y * Math.Sin(Bearing);
            double dyn_posy = kin_x * Math.Sin(Bearing) + kin_y * Math.Cos(Bearing);

            dyn_posx = (Math.Cos(Bearing) - Beta * Math.Sin(Bearing)) * Speed;
            dyn_posy = (Math.Sin(Bearing) + Beta * Math.Cos(Bearing)) * Speed;
            delx = dyn_posx;
            dely = dyn_posy;
            deltheta = Beta;
            delpsi = dyn_psi;
            delv = kin_v;
            delFd = 0;
        }



        /// <summary>
        /// This function computes a new 2D position given an origin and a X and Y axis displacement.
        /// </summary>
        /// <param name="origin">Position before displacement. 2D Point representing the robot position.</param>
        /// <param name="xToMove">X abscissa displacement.</param>
        /// <param name="yToMove">Y abscissa displacement.</param>
        /// <returns>New Point2D representing the new robot position, after it has moved.</returns>
        public Point2D CalculateRobotNextPositionCartesian(Point2D origin, double xToMove, double yToMove)
        {
            return new Point2D(origin.X + xToMove, origin.Y + yToMove);          
        }


        /// <summary>
        /// This function computes a new 2D position given an origin, a yaw (heading) angle and a distance along this angle.
        /// </summary>
        /// <param name="origin">Position before displacement. 2D Point representing the robot position.</param>
        /// <param name="angle">Yaw angle (heading) of the robot. Should be given in radians.</param>
        /// <param name="distance">Distance of displacement, in meters.</param>
        /// <returns>New Point2D representing the new robot position, after it has moved.</returns>
        public Point2D CalculateRobotNextPositionPolar(Point2D origin, double angle, double distance)
        {
            double xToMove = Math.Cos(angle) * distance;
            double yToMove = Math.Sin(angle) * distance;
            return CalculateRobotNextPositionCartesian(origin, xToMove, yToMove);
        }
        
    }
}
