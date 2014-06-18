using System;
using BulletSharp;
using BulletSharp.MultiThreaded;
using DemoFramework;
using System.Windows.Forms;
using System.Collections.Generic;
namespace StickDemo
{
    class StickDemo : Demo
    {
        double Interval = 1.0d;
        int count = 0;
        int pathPtCount = 0;
        BoxShape box = null;
        CompoundShape bComp;
        RigidBody bCompRigidBody;
        RigidBody stickSample;
        Vector3 pathStPt, pathEndPt;
        Vector3[] pos;
        //Variables
        float gravity;
        float friction;
        //stick
        float stickMass;
        float stickSizeX;
        float stickSizeY;
        float stickSizeZ;
        float releaseHt;
        //base
        float baseW;
        float baseD;
        float baseH;
        float baseWallThickness;
        int obstacleX;
        int obstacleY;
        float obstacleW;
        float obstacleD;
        float obstacleH;
        //Path
        float pathStX;
        float pathEndX;
        int pathDiv = 100;

        void dropStickAlongPath(Vector3 pathPt)
        {
            Vector3 pos = pathPt;
            //Vector3 localInertia;
            //box.CalculateLocalInertia(stickMass, out localInertia);
            RigidBody cmbody = LocalCreateRigidBody(stickMass, Matrix.Translation(pos), box);
            cmbody.Friction = friction;
            count++;
        }

        //Similar to draw()
        public override void OnUpdate()
        {
            if (realTime>Interval)
            {
                realTime -= Interval;
                if (run) //When R is pressed, run turns to true. When it is pressed again, run turns to false and dropping sticks stops, but simulations goes.
                {
                    dropStickAlongPath(pos[pathPtCount]);
                    pathPtCount++;
                    if (pathPtCount >= pathDiv)
                    {
                        pathPtCount = 0;
                        for (int i = 0; i < pathDiv; i++)
                        {
                            pos[i].Y += 2;
                        }
                    }
                }
            }
            Graphics.SetInfoText("Press R to start simulation\n" +
                                    "F3 - Toggle debug\n" +
                                    "Space - Shoot box\n"+
                                    "Total boxes:"+count);
            base.OnUpdate();
        }

        //Camera settings
        Vector3 eye = new Vector3(0,150 , -100);
        Vector3 target = new Vector3(0, 30, 0);

        bool UseParallelDispatcherBenchmark = true;   //Best setting in performance

        float defaultContactProcessingThreshold = 0.0f;
        ControlPanel cp5;

        //Similar to setup()
        protected override void OnInitialize()
        {
            float scale = 0.1f;
            cp5 = new ControlPanel();
            cp5.addSlider("baseW", (val) => { baseW = (float)val * scale; renewBase(); }, 1, 2000, 1000, 20, 40);
            cp5.addSlider("baseD", (val) => { baseD = (float)val * scale; renewBase(); }, 1, 1000, 50, 20, 90);
            cp5.addSlider("baseH", (val) => { baseH = (float)val * scale; renewBase(); }, 1, 1000, 150, 20, 140);
            cp5.addSlider("baseWallThickness", (val) => { baseWallThickness = (float)val * scale; renewBase(); }, 1, 30, 20, 20, 190);

            cp5.addSlider("obstacleX", (val) => { obstacleX = val; renewBase(); }, 1, 20, 8, 20, 240);
            cp5.addSlider("obstacleY", (val) => { obstacleY = val; renewBase(); }, 1, 10, 1, 20, 290);
            cp5.addSlider("obstacleW", (val) => { obstacleW = (float)val * scale; renewBase(); }, 1, 50, 10, 20, 340);
            cp5.addSlider("obstacleD", (val) => { obstacleD = (float)val * scale; renewBase(); }, 1, 50, 10, 20, 390);
            cp5.addSlider("obstacleH", (val) => { obstacleH = (float)val * scale; renewBase(); }, 0, 1000, 200, 20, 440);

            cp5.addSlider("stickSizeX", (val) => { stickSizeX = (float)val * scale; renewStick(); }, 1, 50, 5, 20, 490);
            cp5.addSlider("stickSizeY", (val) => { stickSizeY = (float)val * scale; renewStick(); }, 1, 50, 5, 20, 540);
            cp5.addSlider("stickSizeZ", (val) => { stickSizeZ = (float)val * scale; renewStick(); }, 50, 500, 200, 20, 590);

            cp5.Show();
            gravity = 300 * scale;
            friction = 1;
            //stick
            stickMass = 1 * scale;
            releaseHt = 500 * scale;
            //Path
            pathStX = -500 * scale;
            pathEndX = 500 * scale;
            pathDiv = 100;

            Freelook.SetEyeTarget(eye, target);
            Graphics.SetFormText("BulletSharp - Benchmark Demo");
            Graphics.SetInfoText("Press R to start simulation\n" +
                                    "F3 - Toggle debug\n" +
                                    "Space - Shoot box\n" +
                                    "Total boxes:" + count);

            pos = new Vector3[pathDiv];
            pathStPt = new Vector3(pathStX, releaseHt,0);
            pathEndPt = new Vector3(pathEndX, releaseHt,0);
            Vector3 pathPos = new Vector3(pathStPt.X, pathStPt.Y, pathStPt.Z);
            Vector3 pathDir = new Vector3((pathEndPt.X - pathStPt.X) / pathDiv, (pathEndPt.Y - pathStPt.Y) / pathDiv, (pathEndPt.Z - pathStPt.Z) / pathDiv);
            for (int i = 0; i < pathDiv; i++)
            {
                pathPos += pathDir;
                pos[i] = new Vector3(pathPos.X, pathPos.Y, pathPos.Z);
            }
        }

        ThreadSupportInterface CreateSolverThreadSupport(int maxNumThreads)
        {
            //#define SEQUENTIAL
            // if (SEQUENTIAL)
            /*{
                SequentialThreadSupport::SequentialThreadConstructionInfo tci("solverThreads",SolverThreadFunc,SolverlsMemoryFunc);
                SequentialThreadSupport* threadSupport = new SequentialThreadSupport(tci);
                threadSupport->startSPU();
            }*/
            //else 
            // {

            Win32ThreadConstructionInfo threadConstructionInfo = new Win32ThreadConstructionInfo("solverThreads",
                Win32ThreadFunc.SolverThreadFunc, Win32LSMemorySetupFunc.SolverLSMemoryFunc, maxNumThreads);
            Win32ThreadSupport threadSupport = new Win32ThreadSupport(threadConstructionInfo);
            threadSupport.StartSpu();
             
            return threadSupport;
        }

        //Another setup()
        protected override void OnInitializePhysics()
        {
            // collision configuration contains default setup for memory, collision setup
            DefaultCollisionConstructionInfo cci = new DefaultCollisionConstructionInfo();
            cci.DefaultMaxPersistentManifoldPoolSize = 32768*30;
            //cci.DefaultMaxCollisionAlgorithmPoolSize=
            CollisionConf = new DefaultCollisionConfiguration(cci);

            if (UseParallelDispatcherBenchmark)
            {
                int maxNumOutstandingTasks = 4;

                Win32ThreadConstructionInfo info = new Win32ThreadConstructionInfo("collision",
                    Win32ThreadFunc.ProcessCollisionTask, Win32LSMemorySetupFunc.CreateCollisionLocalStoreMemory,
                    maxNumOutstandingTasks);

                Win32ThreadSupport threadSupportCollision = new Win32ThreadSupport(info);
                Dispatcher = new SpuGatheringCollisionDispatcher(threadSupportCollision, 1, CollisionConf);
            }
            else
            {
                Dispatcher = new CollisionDispatcher(CollisionConf);
                Dispatcher.DispatcherFlags = DispatcherFlags.DisableContactPoolDynamicAllocation;
            }

            // the maximum size of the collision world. Make sure objects stay within these boundaries
            // Don't make the world AABB size too large, it will harm simulation quality and performance
            Vector3 worldAabbMin = new Vector3(-1000, -1000, -1000);
            Vector3 worldAabbMax = new Vector3(1000, 1000, 1000);

            HashedOverlappingPairCache pairCache = new HashedOverlappingPairCache();
            
            Broadphase = new DbvtBroadphase(pairCache);
            //Broadphase = new DbvtBroadphase();

            if (UseParallelDispatcherBenchmark)
            {
                ThreadSupportInterface thread = CreateSolverThreadSupport(4);
                Solver = new ParallelConstraintSolver(thread);
            }
            else
            {
                Solver = new SequentialImpulseConstraintSolver();
            }

            World = new DiscreteDynamicsWorld(Dispatcher, Broadphase, Solver, CollisionConf);
            World.Gravity = new Vector3(0, -(gravity),0);

            if (UseParallelDispatcherBenchmark)
            {
                ((DiscreteDynamicsWorld)World).SimulationIslandManager.SplitIslands = false;
            }
            World.SolverInfo.SolverMode |= SolverModes.EnableFrictionDirectionCaching;
            World.SolverInfo.NumIterations = 5;

            // create the ground
            CollisionShape plane = new StaticPlaneShape(new Vector3(0,1,0),0);
            //CollisionShape groundShape = new BoxShape(250, 50, 250);
            CollisionShapes.Add(plane);
            CollisionObject ground = base.LocalCreateRigidBody(0, Matrix.Translation(0, 0, 0), plane);
            ground.UserObject = "Ground";
            bComp = new CompoundShape(true);
            renewBase();
            box = new BoxShape(stickSizeX, stickSizeZ, stickSizeY);
            stickSample = LocalCreateRigidBody(0, Matrix.Translation(new Vector3(-50,0,-30)), box);
            renewStick();
        }
        void renewStick()
        {
            if (box == null) return;
            box = new BoxShape(stickSizeX, stickSizeZ, stickSizeY);
            stickSample.CollisionShape = box;
        }
        void renewBase()
        {
            if (bComp == null) return;
            CollisionShapes.Remove(bComp);
            if (bCompRigidBody != null) World.RemoveRigidBody(bCompRigidBody);
            bComp = new CompoundShape(true);
            bComp.AddChildShape(Matrix.Translation(new Vector3(0, baseH / 2, -(baseD / 2) - (baseWallThickness / 2))), new BoxShape(new Vector3(baseW / 2f + (baseWallThickness), baseH / 2f, baseWallThickness / 2f)));
            bComp.AddChildShape(Matrix.Translation(new Vector3((baseW / 2) + (baseWallThickness / 2), baseH / 2, 0)), new BoxShape(new Vector3(baseWallThickness / 2f, baseH / 2f, baseD / 2f)));
            bComp.AddChildShape(Matrix.Translation(new Vector3(0, baseH / 2, baseD / 2 + (baseWallThickness / 2))), new BoxShape(new Vector3(baseW / 2f + (baseWallThickness), baseH / 2f, baseWallThickness / 2f)));
            bComp.AddChildShape(Matrix.Translation(new Vector3(-(baseW / 2) - (baseWallThickness / 2), baseH / 2, 0)), new BoxShape(new Vector3(baseWallThickness / 2f, baseH / 2f, baseD / 2f)));
            // convert obstacles
            for (int i = 1; i <= obstacleX; i++)
            {
                for (int j = 1; j <= obstacleY; j++)
                {
                    bComp.AddChildShape(Matrix.Translation(new Vector3(((baseW / (obstacleX + 1)) * i) - baseW / 2, obstacleH / 2, ((baseD / (obstacleY + 1)) * j) - baseD / 2)), new BoxShape(new Vector3(obstacleW / 2f, obstacleH / 2f, obstacleD / 2f)));
                }
            }
            CollisionShapes.Add(bComp);
            bCompRigidBody=LocalCreateRigidBody(0, Matrix.Translation(0, 0, 0), bComp);

        }
        public override RigidBody LocalCreateRigidBody(float mass, Matrix startTransform, CollisionShape shape)
        {
            //rigidbody is dynamic if and only if mass is non zero, otherwise static
            bool isDynamic = (mass != 0.0f);

            Vector3 localInertia = Vector3.Zero;
            if (isDynamic)
                shape.CalculateLocalInertia(mass, out localInertia);

            //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

            RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, null, shape, localInertia);
            RigidBody body = new RigidBody(rbInfo);
            rbInfo.Dispose();
            body.ContactProcessingThreshold = defaultContactProcessingThreshold;
            body.WorldTransform = startTransform;

            World.AddRigidBody(body);
            return body;
        }
    }

    static class Program
    {
        [STAThread]
        static void Main()
        {
            using (Demo demo = new StickDemo())
            {
                LibraryManager.Initialize(demo);
            }
        }
    }
}
