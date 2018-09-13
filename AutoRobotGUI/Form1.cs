using AutoRobot;
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
            Point2D position = robot.MoveRobot();
            if (position.X == 0 && position.Y == 0)
                return;
                this.chart1.Series[1].Points.AddXY(position.X, position.Y);

                    pictureBox1.Location = new Point((int)(position.X*ratioX), this.Height-(int)(position.Y*ratioY));
            float robotBearing = (float)(robot.Bearing * 180 / Math.PI);
            
             rotatedImage = RotateImage(new Bitmap(pictureBox1.Image), PreviousBearing - robotBearing);

            PreviousBearing = robotBearing;

            pictureBox1.Image = rotatedImage;
            step++;
            
        }

        public void ResizeEvent(Object myObject,
                                           EventArgs myEventArgs)
        {
            this.PerformAutoScale();
            this.chart1.Width = this.Width;
            this.chart1.Height = this.Height;
        }
        public Form1()
        {
            
            InitializeComponent();
            WindowState = FormWindowState.Maximized;

            Random rand;
            double x, y;


            rand = new Random();

            int numberWayPoints = rand.Next(15, 20);

            double maxX = -1000000;
            double maxY = -1000000;
            double minX = 100000000;
            double minY = 100000000;
            List<Point2D> WayPoints = new List<Point2D>();
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
                Point2D newWayPoint = robot.CalculateRobotNextPositionPolar(WayPoints[i - 1], Point2D.AbsoluteBearing(WayPoints[i - 2], WayPoints[i - 1]) + changAngle, distBetweenWayPoints);
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

            this.chart1.ChartAreas[0].AxisX.Maximum = maxX + 5;
            this.chart1.ChartAreas[0].AxisX.Minimum = minX - 5;
            this.chart1.ChartAreas[0].AxisY.Maximum = maxY + 5;
            this.chart1.ChartAreas[0].AxisY.Minimum = minY - 5;

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


            myTimer.Tick += new EventHandler(TimerEventProcessor);

            // Sets the timer interval to 5 seconds.
            myTimer.Interval = 50;
            myTimer.Start();


           // myTimer.Stop();
        }

        private void chart1_Click(object sender, EventArgs e)
        {

        }
    }
}
