using AutoRobot;
using AutoRobotExercises;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace AutoRobotGUI
{
    public partial class Form1 : Form
    {
        static System.Windows.Forms.Timer myTimer = new System.Windows.Forms.Timer();
        public Robot robot { get; set; }
        public float PreviousBearing = (float)(0.6*180/Math.PI);
        double ratioX;
        double ratioY;
        List<Point2D> ObstaclesPositions;
        List<Point2D> WayPoints;
        double ObstSize = 2.0;
        double u1 = 2.8, u2 = 2.8;
        double ConsigneVitesse = 0.4;
        double K = 0.1;

        private Bitmap RotateImage(Bitmap bmp, float angle)
        {
            Bitmap rotatedImage = new Bitmap(bmp.Width, bmp.Height);
            using (Graphics g = Graphics.FromImage(rotatedImage))
            {
                // Set the rotation point to the center in the matrix
                g.TranslateTransform(bmp.Width / 2, bmp.Height / 2);
                // Rotate
                g.RotateTransform(angle);
                // Restore rotation point in the matrix
                g.TranslateTransform(-bmp.Width / 2, -bmp.Height / 2);
                // Draw the image on the bitmap
                g.DrawImage(bmp, new Point(0, 0));
            }

            return rotatedImage;
        }

        
        int step = 0; 
        public void TimerEventProcessor(Object myObject,
                                            EventArgs myEventArgs)
        {
            this.chart1.Series[2].Points.Clear();
            for (int i = 0; i < ObstaclesPositions.Count; i++)
            {
                this.chart1.Series[2].Points.AddXY(ObstaclesPositions[i].X, ObstaclesPositions[i].Y);
                this.chart1.Series[2].Points[i].MarkerSize = (int)ObstSize*250;
                this.chart1.Series[2].Points[i].MarkerStyle = System.Windows.Forms.DataVisualization.Charting.MarkerStyle.Circle;
            }
            Bitmap rotatedImage;
            if (step == 0)
            {
                while (pictureBox1.Image == null)
                {
                    Thread.Sleep(50);
                }
                 rotatedImage = RotateImage(new Bitmap(pictureBox1.Image), (float)((0.6 + Math.PI/2)* 180 / Math.PI));
                pictureBox1.Image = rotatedImage;
                 
            }
            double RegulatedVoltageU1;
            double RegulatedVoltageU2;
            // ----------------------------------------- MOVEROBOT ---------------------------------
            Point2D position = robot.MoveRobot(out RegulatedVoltageU1, out RegulatedVoltageU2, ConsigneVitesse, 0.6, K);
            
            if (position.X == 0 && position.Y == 0)
                return;
                this.chart1.Series[1].Points.AddXY(position.X, position.Y);

                    pictureBox1.Location = new Point((int)(position.X*ratioX), this.Height-(int)(position.Y*ratioY));
            float robotBearing = (float)(robot.Bearing * 180 / Math.PI);
            
             rotatedImage = RotateImage(new Bitmap(pictureBox1.Image), PreviousBearing - robotBearing);

            PreviousBearing = robotBearing;

            pictureBox1.Image = rotatedImage;
            /*
            this.chart1.Series[0].Points.Clear();
            List<List<Point2D>> corridor = robot.GetVehicleCorridor(robot.GetEstimatedFuturTrajectory(8));
            for (int i = 0; i < corridor.Count; i++)
            {
                for (int j = 0; j < corridor[i].Count; j++)
                {
                    this.chart1.Series[0].Points.AddXY(corridor[i][j].X, corridor[i][j].Y);
                }
            }
            */
            step++;
            this.chart1.Series[0].Points.Clear();
            
            
            int VehGridX, VehGridY;
            if (robot.IsObstacleInTrajectory(ObstaclesPositions, 10))
            {
                int[,] Grid = robot.CreatePotentialField(ObstaclesPositions, ObstSize, out VehGridX, out VehGridY, 50, 50, ObstSize, 100);
                List<Point2D> positions = robot.FindShortestPath(Grid, 50, 50, VehGridX, VehGridY);
                if (positions != null)
                {
                    if (positions.Count != 0) // Found a trajectory
                    {
                        int startIdx, endIdx;
                        Robot dummyRobot = new Robot(robot.Position, robot.Bearing);
                        dummyRobot.Trajectory = new List<Point2D>(robot.Trajectory);
                        dummyRobot.MinIdxTraj = robot.MinIdxTraj;
                        dummyRobot.Position = robot.Position;
                        dummyRobot.FindTrajectorySegment(0, out startIdx);
                        dummyRobot.Position = positions[positions.Count - 1];
                        dummyRobot.FindTrajectorySegment(0, out endIdx, dummyRobot.Trajectory.Count);

                        int numberToSuppress = endIdx - startIdx;
                        while (numberToSuppress > 0)
                        {
                            robot.Trajectory.RemoveAt(startIdx);
                            numberToSuppress--;
                        }
                        robot.Trajectory.InsertRange(startIdx, positions);

                        robot.Trajectory = robot.CalculateIntermediatePointsBetweenWayPoints(robot.Trajectory, 0.2);
                    }
                    
                }
                else
                {
                        if (robot.Speed > 0)
                        {
                            // Speed we want to reach
                            ConsigneVitesse = 0;

                            // How strongly we want to brake [0,+Inf]
                            K = 10;
                        }
                        else
                        {
                            K = 0;
                            robot.InputVoltage1 = 0;

                            // Speed cannot be negative
                            robot.Speed = 0; 
                        }
                    
                }
            }
            else
            {
                int j = 2;
            }

            this.chart1.Series[0].Points.Clear();
            for (int i = 0; i < robot.Trajectory.Count; i++)
            {
                this.chart1.Series[0].Points.AddXY(robot.Trajectory[i].X, robot.Trajectory[i].Y);
            }
            
        }

        public void ResizeEvent(Object myObject,
                                           EventArgs myEventArgs)
        {
            this.chart1.Location = new Point(0, 0);
            //this.PerformAutoScale();
            this.chart1.Width = this.Width;
            this.chart1.Height = this.Height;
        }
        public Form1()
        {
            
            InitializeComponent();
            WindowState = FormWindowState.Maximized;
            
            Random rand;
            double x, y;


            rand = new Random(5);



            int numberWayPoints = 3;//rand.Next(15,20);

            double maxX = -1000000;
            double maxY = -1000000;
            double minX = 100000000;
            double minY = 100000000;
            WayPoints = new List<Point2D>();
            double distBetweenWayPoints = 10;
            //x = rand.NextDouble() * 10;
            //y = rand.NextDouble() * 10;
            x = 5;
            y = 5;
            Point2D wayPoint = new Point2D(x, y);
            WayPoints.Add(wayPoint);
            this.robot = new Robot(WayPoints[0], 0.6);

            pictureBox1.ImageLocation = "C:\\Users\\Nicolas\\Documents\\car-png-top-view-png-white-top-car-png-image-34867-587-resize.png";
            pictureBox1.SizeMode = PictureBoxSizeMode.AutoSize;
            //pictureBox1.ImageLocation = "http://pluspng.com/img-png/car-png-top-view-png-white-top-car-png-image-34867-587.png";

            

            wayPoint = robot.CalculateRobotNextPositionPolar(WayPoints[0], 0.6, distBetweenWayPoints);
            WayPoints.Add(wayPoint);
            for (int i = 2; i < numberWayPoints; i++)
            {
                double changAngle = rand.NextDouble() *3;
                //if (i % 2 == 0)
                changAngle *= -1;
                Point2D newWayPoint = robot.CalculateRobotNextPositionPolar(WayPoints[i - 1], WayPoints[i - 2].AbsoluteBearing( WayPoints[i - 1]) + changAngle, distBetweenWayPoints);
                WayPoints.Add(newWayPoint);
           
                    
            }

            for (int i = 0; i < numberWayPoints; i++)
            {
                x = WayPoints[i].X;
                y = WayPoints[i].Y;
                if (x > maxX)
                    maxX = x;
                if (x < minX)
                    minX = x;
                if (y > maxY)
                    maxY = y;
                if (y < minY)
                    minY = y;
            }


            ratioX = (this.Width + 40) / (maxX - minX);
            ratioY = (this.Height + 20) / (maxY - minY);
            pictureBox1.Location = new Point((int)(x * ratioX), (int)(y * ratioY));

            this.chart1.ChartAreas[0].AxisX.Maximum = maxX;
            this.chart1.ChartAreas[0].AxisX.Minimum = minX;
            this.chart1.ChartAreas[0].AxisY.Maximum = maxY;
            this.chart1.ChartAreas[0].AxisY.Minimum = minY;

            this.Resize += ResizeEvent; 
            List<Point2D> trajectory = robot.CalculateIntermediatePointsBetweenWayPoints(WayPoints, 0.2);
            robot.Trajectory = trajectory;
            this.chart1.Series[0].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Point;
            this.chart1.Series[0].Points.Clear();
            this.chart1.Series[0].Name = "PlannedTrajectory";
            for (int i = 0; i < trajectory.Count; i++)
                this.chart1.Series[0].Points.AddXY(trajectory[i].X, trajectory[i].Y);

            
            this.chart1.Series.Add(new System.Windows.Forms.DataVisualization.Charting.Series("RobotMovement"));
            this.chart1.Series[1].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Point;
            this.chart1.Series.Add(new System.Windows.Forms.DataVisualization.Charting.Series("Obstacles"));
            this.chart1.Series[2].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Point;
            this.chart1.Series.Add(new System.Windows.Forms.DataVisualization.Charting.Series("OriginalTraj"));
            this.chart1.Series[3].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Point;

            for (int i = 0; i < trajectory.Count; i++)
                this.chart1.Series[3].Points.AddXY(trajectory[i].X, trajectory[i].Y);
            myTimer.Tick += new EventHandler(TimerEventProcessor);

            // Sets the timer interval to 5 seconds.
            myTimer.Interval = 50;
            myTimer.Start();
            pictureBox1.Visible = false;
            ObstaclesPositions = new List<Point2D>();
            ObstaclesPositions.Add(WayPoints[2]);
            ObstaclesPositions.Add(WayPoints[1]);
            //ObstaclesPositions.Add(new Point2D(WayPoints[1].X,6.25));


            // myTimer.Stop();
        }

        private void chart1_Click(object sender, EventArgs e)
        {
            double XChart = chart1.ChartAreas[0].AxisX.PixelPositionToValue(MousePosition.X);
            double YChart = chart1.ChartAreas[0].AxisY.PixelPositionToValue(MousePosition.Y);
            this.ObstaclesPositions.Add(new Point2D(XChart, YChart));
        }
    }
}
