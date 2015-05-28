using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Fusion;
using Fusion.Mathematics;
using Fusion.Graphics;
using Fusion.Audio;
using Fusion.Input;
using Fusion.Content;
using Fusion.Development;

using BEPUphysics;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.Vehicle;
using BEPUutilities;
using Vector3BEPU = BEPUutilities.Vector3;
using MatrixBEPU = BEPUutilities.Matrix;
using BEPUphysics.BroadPhaseEntries;
using Vector3Fusion = Fusion.Mathematics.Vector3;
using Vector4Fusion = Fusion.Mathematics.Vector4;
using Quaternion = BEPUutilities.Quaternion;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Constraints.SolverGroups;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.SingleEntity;
using BEPUphysics.CollisionRuleManagement;

namespace SkeletonLib
{
    public struct SkeletonNode
    {
        public List<SkeletonNode> childs;
        public int depth;
        public String Name;
    }

    public class BaseSkeleton
    {
        public String[] bodyNodes = { "center", "thigh_l", "thigh_r", "spine", "knee_l", "knee_r", "neck", "ankle_l", "ankle_r", "shoulder_l", "head", "shoulder_r", "foot_l", "foot_r", "elbow_l", "elbow_r", "wrist_l", "wrist_r" };
        public Dictionary<String, MatrixBEPU> startPose, endPose;
        public Space space;
        public List<KeyValuePair<String, String>> skeletonConnections;
        public SkeletonNode mainBodyNode;
        public List<AngularMotor> angularMotors;

        public BaseSkeleton(Space space) {
            this.space = space;
            initPoses();
            initBoxes();
            initSkeleton();
        }

        //17 + 1 = 18
        public double[] getModelInput(double time)
        {
            double maxCoord = 20;
            double minCoord = -20;

            double[] ans = new double[bodyNodes.Length * 3 + 2];
            ans[bodyNodes.Length * 3] = time;
            for (int i = 0; i < bodyNodes.Length; i++) {
                Entity entity = findBox(bodyNodes[i]);
                Vector3BEPU position = entity.Position;
                ans[3 * i] = (position.X - minCoord) / (maxCoord - minCoord);
                ans[3 * i + 1] = (position.Y - minCoord) / (maxCoord - minCoord);
                ans[3 * i + 2] = (position.Z - minCoord) / (maxCoord - minCoord);
            }
            ans[bodyNodes.Length * 3 + 1] = 1;

            return ans;
        }

        public void setMotorInput2(double[] input)
        {
            if (input.Length != angularMotors.Count() * 3)
            {
                Console.WriteLine("input wrong size: " + input.Length + " and " + angularMotors.Count());
            }

            int i = 0;
            foreach (AngularMotor angularMotor in angularMotors)
            {
               // angularMotor.Settings.VelocityMotor.GoalVelocity = new Vector3BEPU((float)(2 * input[i] - 1), (float)(2 * input[i + 1] - 1), (float)(2 * input[i + 2] - 1));
                angularMotor.Settings.VelocityMotor.GoalVelocity = new Vector3BEPU((float)(2 * input[i] - 1) / 15, (float)(2 * input[i + 1] - 1) / 15, (float)(2 * input[i + 2] - 1) / 15);
                i += 3;
            }
        }

        public void setMotorInput(double[] input)
        {
            if (input.Length != angularMotors.Count() * 3) {
                Console.WriteLine("input wrong size: " + input.Length + " and " + angularMotors.Count());
            }

            int i = 0;
            foreach (AngularMotor angularMotor in angularMotors)
            {
                float yaw = (float)((2 * input[i] - 1) * Math.PI / 1.5);
                float pitch = (float)((2 * input[i + 1] - 1) * Math.PI / 1.5);
                float roll = (float)((2 * input[i + 2] - 1) * Math.PI / 3);
                angularMotor.Settings.Servo.Goal = Quaternion.CreateFromYawPitchRoll(yaw, pitch, roll);
                i += 3;
            }
        }

        public double getDiffMotor(double[] input) {
            int i = 0;
            double ans = 0;
            foreach (AngularMotor angularMotor in angularMotors)
            {
                float yaw = (float)((2 * input[i] - 1) * Math.PI);
                float pitch = (float)((2 * input[i + 1] - 1) * Math.PI);
                float roll = (float)((2 * input[i + 2] - 1) * Math.PI / 2);
                Quaternion q = Quaternion.CreateFromYawPitchRoll(yaw, pitch, roll);
                Quaternion q2 = angularMotor.Settings.Servo.Goal;
                double temp = Math.Pow(q.W - q2.W, 2) + Math.Pow(q.X - q2.X, 2) + Math.Pow(q.Y - q2.Y, 2) + Math.Pow(q.Z - q2.Z, 2);
                ans += Math.Pow(temp, 2);
                i += 3;
            }
            return ans;
        }

        public double getDifferenceBetweenCurrentAndEnd()
        {
            double ans = 0;
            foreach (String nodeName in bodyNodes)
            {
                Vector3BEPU vCurrent = findBox(nodeName).Position;
                Vector3BEPU vPose = endPose[nodeName].Translation;
                vPose.X += 4;
                Vector3BEPU temp = vCurrent - vPose;
                ans += Math.Pow(temp.Length(), 2);
            }

            return ans;
        }

        private void initBox(String name, MatrixBEPU position)
        {
            if (!bodyNodes.Contains(name)) { return; }
            Box box = new Box(new Vector3BEPU(0, 0, 0), 1f, 1f, 1f, 10f);
            box.WorldTransform = position;
            box.Tag = name;
            space.Add(box);
        }

        private void initBoxes() {
            // add ground and enviroment
            space.ForceUpdater.Gravity = new Vector3BEPU(0, -6f, 0);
            Box ground = new Box(new Vector3BEPU(0, -2.5f, 0), 50, 1, 50);
            space.Add(ground);

            // add boxes
            foreach (String nodeName in bodyNodes) {
                initBox(nodeName, startPose[nodeName]);
            }
        }

        private void initPoses()
        {
            startPose = new Dictionary<string, MatrixBEPU>();
            startPose.Add("RootNode", new MatrixBEPU(1f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 0f, 1f));
            startPose.Add("pCube1", new MatrixBEPU(1f, 0f, 0f, 0f, 0f, 5f, 0f, 0f, 0f, 0f, 1f, 0f, -1f, 7f, 0f, 1f));
            startPose.Add("pCube2", new MatrixBEPU(1f, 0f, 0f, 0f, 0f, 5f, 0f, 0f, 0f, 0f, 1f, 0f, 1f, 7f, 0f, 1f));
            startPose.Add("pCube3", new MatrixBEPU(1f, 0f, 0f, 0f, 0f, 4f, 0f, 0f, 0f, 0f, 1f, 0f, -1f, 2f, 0f, 1f));
            startPose.Add("pCube4", new MatrixBEPU(1f, 0f, 0f, 0f, 0f, 4f, 0f, 0f, 0f, 0f, 1f, 0f, 1f, 2f, 0f, 1f));
            startPose.Add("pCube5", new MatrixBEPU(1f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 0f, 2f, 0f, -1f, -1f, 0.2f, 1f));
            startPose.Add("pCube6", new MatrixBEPU(1f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 0f, 2f, 0f, 1f, -1f, 0.2f, 1f));
            startPose.Add("pCube7", new MatrixBEPU(3.3f, 0f, 0f, 0f, 0f, 1.710814f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 10.69011f, 0f, 1f));
            startPose.Add("pCube8", new MatrixBEPU(3.1f, 0f, 0f, 0f, 0f, 2.138517f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 12.82145f, 0f, 1f));
            startPose.Add("pCube9", new MatrixBEPU(3.5f, 0f, 0f, 0f, 0f, 2.138517f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 15.18839f, 0f, 1f));
            startPose.Add("pCube10", new MatrixBEPU(2f, 0f, 0f, 0f, 0f, 2f, 0f, 0f, 0f, 0f, 2f, 0f, -0.0003992501f, 18f, 0f, 1f));
            startPose.Add("pCube11", new MatrixBEPU(3.2f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 0f, 1f, 0f, -3.7f, 15.8f, 0f, 1f));
            startPose.Add("pCube12", new MatrixBEPU(3.2f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 0f, 1f, 0f, 3.7f, 15.8f, 0f, 1f));
            startPose.Add("pCube13", new MatrixBEPU(3f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 0f, 1f, 0f, -7.2f, 15.8f, 0f, 1f));
            startPose.Add("pCube14", new MatrixBEPU(3.2f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 0f, 1f, 0f, 7.2f, 15.8f, 0f, 1f));
            startPose.Add("center", new MatrixBEPU(0.4762832f, -0.879292f, 0f, 0f, 0.879292f, 0.4762832f, 0f, 0f, 0f, 0f, 1f, 0f, -0.01556377f, 11.6482f, -0.1053383f, 1f));
            startPose.Add("pCube15", new MatrixBEPU(1f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 0f, 1f, 0f, -1f, -1f, 2f, 1f));
            startPose.Add("pCube16", new MatrixBEPU(1f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 0f, 1f, 0f, 1f, -1f, 2f, 1f));
            startPose.Add("thigh_l", new MatrixBEPU(-0.005025062f, -0.9999874f, -1.224647E-16f, 0f, -0.9999874f, 0.005025062f, -9.62965E-35f, 0f, 6.153926E-19f, 1.224631E-16f, -1f, 0f, 1.064769f, 9.65374f, 0.1064478f, 1f));
            startPose.Add("thigh_r", new MatrixBEPU(-0.005025062f, 0.9999874f, 0f, 0f, -0.9999874f, -0.005025062f, 2.107342E-08f, 0f, 2.107316E-08f, 1.058953E-10f, 1f, 0f, -1.095893f, 9.65374f, 0.1064478f, 1f));
            startPose.Add("spine", new MatrixBEPU(-0.0104161f, 0.9999458f, 0f, 0f, 0.9999458f, 0.0104161f, 1.224647E-16f, 0f, 1.22458E-16f, 1.275605E-18f, -1f, 0f, 0.01213706f, 14.05535f, -0.1053383f, 1f));
            startPose.Add("knee_l", new MatrixBEPU(-0.006289184f, -0.9999802f, -1.224647E-16f, 0f, -0.9999802f, 0.006289184f, 0f, 0f, 7.702029E-19f, 1.224623E-16f, -1f, 0f, 1.037068f, 4.24876f, 0.2915335f, 1f));
            startPose.Add("knee_r", new MatrixBEPU(-0.006289184f, 0.9999802f, -4.717878E-26f, 0f, -0.9999802f, -0.006289184f, 5.439343E-08f, 0f, 5.439236E-08f, 3.420903E-10f, 1f, 0f, -1.068193f, 4.270253f, 0.2915335f, 1f));
            startPose.Add("neck", new MatrixBEPU(0.9843366f, -0.1762991f, 0f, 0f, 0.1762991f, 0.9843366f, 0f, 0f, 0f, 0f, 1f, 0f, -0.01390319f, 16.15245f, -0.1053383f, 1f));
            startPose.Add("ankle_l", new MatrixBEPU(-9.338353E-16f, -0.5444708f, 0.8387798f, 0f, -2.765127E-16f, 0.8387798f, 0.5444708f, 0f, -1f, 2.765127E-16f, -9.338353E-16f, 0f, 0.9799383f, -0.4384661f, -0.396419f, 1f));
            startPose.Add("ankle_r", new MatrixBEPU(-0.9449179f, 0.06359895f, 0.3210692f, 0f, 0.06021769f, 0.9979755f, -0.02046108f, 0f, -0.3217205f, -2.522524E-11f, -0.9468347f, 0f, -1.069922f, -0.4384662f, -0.396419f, 1f));
            startPose.Add("shoulder_l", new MatrixBEPU(0.1897654f, -0.9818295f, 0f, 0f, 0.9818295f, 0.1897654f, 0f, 0f, 0f, 0f, 1f, 0f, 1.88f, 15.9f, -0.1053383f, 1f));
            startPose.Add("head", new MatrixBEPU(0.9843366f, -0.1762991f, 0f, 0f, 0.1762991f, 0.9843366f, 0f, 0f, 0f, 0f, 1f, 0f, -0.03447142f, 17.44367f, -0.1053383f, 1f));
            startPose.Add("shoulder_r", new MatrixBEPU(0.1897654f, 0.9818295f, -3.081488E-33f, 0f, 0.9818295f, -0.1897654f, -1.224647E-16f, 0f, -1.202394E-16f, 2.323955E-17f, -1f, 0f, -1.88f, 15.9f, 0.1f, 1f));
            startPose.Add("foot_l", new MatrixBEPU(1f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 0f, 1f, 0f, 0.9799383f, -1.267311f, 2.228829f, 1f));
            startPose.Add("foot_r", new MatrixBEPU(1f, 0f, 0f, 0f, 0f, -1f, 2.663349E-11f, 0f, 0f, -2.663349E-11f, -1f, 0f, -1.118843f, -1.290949f, 2.216528f, 1f));
            startPose.Add("elbow_l", new MatrixBEPU(-0.002029877f, -0.9999979f, -1.224647E-16f, 0f, -0.9999979f, 0.002029877f, -4.814825E-35f, 0f, 2.485882E-19f, 1.224644E-16f, -1f, 0f, 5.5f, 15.8f, -0.1f, 1f));
            startPose.Add("elbow_r", new MatrixBEPU(-0.002029877f, 0.9999979f, 0f, 0f, -0.9999979f, -0.002029877f, -2.107342E-08f, 0f, -2.107338E-08f, -4.277646E-11f, 1f, 0f, -5.5f, 15.8f, -0.1f, 1f));
            startPose.Add("wrist_l", new MatrixBEPU(0.4762832f, -0.879292f, 0f, 0f, 0.879292f, 0.4762832f, 0f, 0f, 0f, 0f, 1f, 0f, 8.8848f, 15.8f, -0.1053383f, 1f));
            startPose.Add("wrist_r", new MatrixBEPU(0.4762832f, 0.879292f, 0f, 0f, 0.879292f, -0.4762832f, -1.224647E-16f, 0f, -1.076822E-16f, 5.832786E-17f, -1f, 0f, -8.885f, 15.8f, -0.1f, 1f));

            endPose = new Dictionary<string,MatrixBEPU>();

            foreach (String nodeName in bodyNodes) {
                //Vector3BEPU v = startPose[nodeName];
                endPose.Add(nodeName, startPose[nodeName]);
                //endPose.Add(nodeName, new Vector3BEPU(v.X, v.Y, v.Z + 20));
            }
        }

        private void initSkeleton()
        {
            skeletonConnections = new List<KeyValuePair<String, String>>();
            angularMotors = new List<AngularMotor>();

            skeletonConnections.Add(new KeyValuePair<String, String>("center", "thigh_l"));
            skeletonConnections.Add(new KeyValuePair<String, String>("thigh_l", "knee_l"));
            skeletonConnections.Add(new KeyValuePair<String, String>("knee_l", "ankle_l"));
            skeletonConnections.Add(new KeyValuePair<String, String>("ankle_l", "foot_l"));

            skeletonConnections.Add(new KeyValuePair<String, String>("center", "thigh_r"));
            skeletonConnections.Add(new KeyValuePair<String, String>("thigh_r", "knee_r"));
            skeletonConnections.Add(new KeyValuePair<String, String>("knee_r", "ankle_r"));
            skeletonConnections.Add(new KeyValuePair<String, String>("ankle_r", "foot_r"));

            skeletonConnections.Add(new KeyValuePair<String, String>("center", "spine"));
            skeletonConnections.Add(new KeyValuePair<String, String>("spine", "neck"));
            skeletonConnections.Add(new KeyValuePair<String, String>("neck", "head"));

            skeletonConnections.Add(new KeyValuePair<String, String>("neck", "shoulder_l"));
            skeletonConnections.Add(new KeyValuePair<String, String>("shoulder_l", "elbow_l"));
            skeletonConnections.Add(new KeyValuePair<String, String>("elbow_l", "wrist_l"));
            skeletonConnections.Add(new KeyValuePair<String, String>("neck", "shoulder_r"));
            skeletonConnections.Add(new KeyValuePair<String, String>("shoulder_r", "elbow_r"));
            skeletonConnections.Add(new KeyValuePair<String, String>("elbow_r", "wrist_r"));


            mainBodyNode = initSkeletonNode("center", 1);
        }

        private SkeletonNode initSkeletonNode(String nodeName, int depth)
        {
            SkeletonNode node = new SkeletonNode();
            node.Name = nodeName;
            node.depth = depth;
            node.childs = new List<SkeletonNode>();
            Entity box1 = findBox(nodeName);

            foreach (KeyValuePair<String, String> pair in skeletonConnections)
            {
                if (pair.Key.Equals(node.Name))
                {
                    SkeletonNode childNode = initSkeletonNode(pair.Value, depth + 1);
                    node.childs.Add(childNode);

                    Entity box2 = findBox(childNode.Name);

                    BallSocketJoint joint = new BallSocketJoint(box1, box2, box1.Position);
                    joint.SpringSettings.Damping /= 80;
                    joint.SpringSettings.Stiffness /= 80;
                    space.Add(joint);

                    AngularMotor motor = new AngularMotor(box1, box2);
                    motor.Settings.Mode = MotorMode.Servomechanism;

                    BEPUutilities.Quaternion q1 = box1.Orientation;
                    BEPUutilities.Quaternion q2 = box2.Orientation;
                    BEPUutilities.Quaternion qResult;
                    BEPUutilities.Quaternion qTemp;

                    Quaternion.Conjugate(ref q1, out qTemp);
                    Quaternion.Concatenate(ref q2, ref qTemp, out qResult);

                    motor.Settings.Servo.Goal = qResult;

                    motor.Settings.MaximumForce = 1500.0f;
                    motor.Settings.Servo.SpringSettings.Damping /= 30;
                    motor.Settings.Servo.SpringSettings.Stiffness /= 30;
                    //motor.Settings.Mode = MotorMode.VelocityMotor;
                    //motor.Settings.VelocityMotor.GoalVelocity = new Vector3BEPU();
                    space.Add(motor);

                    angularMotors.Add(motor);
                }
            }

            return node;
        }

        public Entity findBox(String tagName)
        {
            foreach (var e in space.Entities)
            {
                Entity box = e as Entity;
                if (box != null && box.Tag != null && box.Tag.Equals(tagName))
                {
                    return box;
                }
            }
            return null;
        }

        /*
        private SkeletonNode initSkeletonNodeWithFixedAngular(String nodeName, int depth)
        {
            SkeletonNode node = new SkeletonNode();
            node.Name = nodeName;
            node.depth = depth;
            node.childs = new List<SkeletonNode>();
            Entity box1 = findBox(nodeName);

            foreach (KeyValuePair<String, String> pair in skeletonConnections)
            {
                if (pair.Key.Equals(node.Name))
                {
                    SkeletonNode childNode = initSkeletonNode(pair.Value, depth + 1);
                    node.childs.Add(childNode);

                    Entity box2 = findBox(childNode.Name);

                    BallSocketJoint joint = new BallSocketJoint(box1, box2, box1.Position);
                    joint.SpringSettings.Damping /= 80;
                    joint.SpringSettings.Stiffness /= 80;
                    space.Add(joint);

                    AngularMotor motor = new AngularMotor(box1, box2);
                    motor.Settings.Mode = MotorMode.Servomechanism;

                    BEPUutilities.Quaternion q1 = box1.Orientation;
                    BEPUutilities.Quaternion q2 = box2.Orientation;
                    BEPUutilities.Quaternion qResult;
                    BEPUutilities.Quaternion qTemp;

                    Quaternion.Conjugate(ref q1, out qTemp);
                    Quaternion.Concatenate(ref q2, ref qTemp, out qResult);

                    motor.Settings.Servo.Goal = qResult;

                    motor.Settings.MaximumForce = 1500.0f;
                    motor.Settings.Servo.SpringSettings.Damping /= 30;
                    motor.Settings.Servo.SpringSettings.Stiffness /= 30;
                    space.Add(motor);

                    angularMotors.Add(motor);
                }
            }

            return node;
        }*/
    }
}
