using System;
using BulletSharp;
using BulletSharp.MultiThreaded;
using DemoFramework;
using System.Windows.Forms;
using System.Collections.Generic;
using grendgine_collada;
using System.IO;
using System.Xml;
using System.Xml.Serialization;
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
        List<RigidBody> sticks = new List<RigidBody>();
        List<RigidBody> frozenSticks = new List<RigidBody>();
        void export()
        {
            Grendgine_Collada col_scenes = null;
            col_scenes = new Grendgine_Collada();
            col_scenes.Collada_Version = "1.4.1";
            col_scenes.Asset = new Grendgine_Collada_Asset();
            col_scenes.Asset.Contributor = new Grendgine_Collada_Asset_Contributor[1];
            col_scenes.Asset.Contributor[0] = new Grendgine_Collada_Asset_Contributor();
            col_scenes.Asset.Contributor[0].Author = "G30";
            col_scenes.Asset.Contributor[0].Authoring_Tool = "something";
            col_scenes.Asset.Created = System.DateTime.Now;
            col_scenes.Asset.Modified = System.DateTime.Now;
            col_scenes.Asset.Unit = new Grendgine_Collada_Asset_Unit();
            col_scenes.Asset.Unit.Meter = 0.025400;
            col_scenes.Asset.Unit.Name = "centimeter";
            col_scenes.Asset.Up_Axis = "Y_UP";
            col_scenes.Library_Geometries = new Grendgine_Collada_Library_Geometries();
            List<RigidBody> sticksAll = new List<RigidBody>();
            sticksAll.AddRange(sticks);
            sticksAll.AddRange(frozenSticks);
            col_scenes.Library_Geometries.Geometry = new Grendgine_Collada_Geometry[sticksAll.Count];
            for (int i = 0; i < sticksAll.Count; i++)
            {
                col_scenes.Library_Geometries.Geometry[i] = new Grendgine_Collada_Geometry();
                Grendgine_Collada_Geometry geom = col_scenes.Library_Geometries.Geometry[i];
                string uniqueID = "Stick"+i.ToString();
                geom.ID = uniqueID + "-lib";
                geom.Name = uniqueID + "Mesh";
                geom.Mesh = new Grendgine_Collada_Mesh();
                geom.Mesh.Source = new Grendgine_Collada_Source[1];
                for (int j = 0; j < 1; j++)
                {
                    geom.Mesh.Source[j] = new Grendgine_Collada_Source();
                }
                geom.Mesh.Source[0].ID = uniqueID + "-POSITION";
                geom.Mesh.Source[0].Float_Array = new Grendgine_Collada_Float_Array();
                geom.Mesh.Source[0].Float_Array.ID = uniqueID + "-POSITION-array";
                //Positions of Vertices
                //ConvexShape
                int nV=0;
                int nF=0;
                if (sticksAll[i].CollisionShape.IsPolyhedral&&sticksAll[i].CollisionShape.IsConvex)
                {
                    PolyhedralConvexShape shape = (PolyhedralConvexShape)sticksAll[i].CollisionShape;
                    nV=shape.VertexCount;
                    nF=shape.PlaneCount;
                    geom.Mesh.Source[0].Float_Array.Count = nV * 3;
                    float[][] vertices = new float[8][];
                    vertices[0] = new float[3] { -stickSizeX / 2f, -stickSizeZ / 2f, -stickSizeY / 2f };
                    vertices[1] = new float[3] { -stickSizeX / 2f, -stickSizeZ / 2f, stickSizeY / 2f };
                    vertices[2] = new float[3] { stickSizeX / 2f, -stickSizeZ / 2f, -stickSizeY / 2f };
                    vertices[3] = new float[3] { stickSizeX / 2f, -stickSizeZ / 2f, stickSizeY / 2f };
                    vertices[4] = new float[3] { -stickSizeX / 2f, stickSizeZ / 2f, -stickSizeY / 2f };
                    vertices[5] = new float[3] { -stickSizeX / 2f, stickSizeZ / 2f, stickSizeY / 2f };
                    vertices[6] = new float[3] { stickSizeX / 2f, stickSizeZ / 2f, -stickSizeY / 2f };
                    vertices[7] = new float[3] { stickSizeX / 2f, stickSizeZ / 2f, stickSizeY / 2f };
                    string val = "";
                    for (int j = 0; j < 8; j++)
                    {
                        for (int k = 0; k < 3; k++)
                        {
                            val += "  "+vertices[j][k].ToString();
                        }
                    }
                    geom.Mesh.Source[0].Float_Array.Value_As_String = val;// "0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0 0.0 1.0 1.0 0.0 0.0 0.0 1.0 1.0 0.0 1.0 0.0 1.0 1.0 1.0 1.0 1.0";
                }
                geom.Mesh.Source[0].Technique_Common = new Grendgine_Collada_Technique_Common_Source();
                geom.Mesh.Source[0].Technique_Common.Accessor = new Grendgine_Collada_Accessor();
                geom.Mesh.Source[0].Technique_Common.Accessor.Source = "#" + geom.Mesh.Source[0].Float_Array.ID;
                geom.Mesh.Source[0].Technique_Common.Accessor.Stride = 3;
                geom.Mesh.Source[0].Technique_Common.Accessor.Count = (uint)geom.Mesh.Source[0].Float_Array.Count / geom.Mesh.Source[0].Technique_Common.Accessor.Stride;
                geom.Mesh.Source[0].Technique_Common.Accessor.Param = new Grendgine_Collada_Param[3];
                geom.Mesh.Source[0].Technique_Common.Accessor.Param[0] = new Grendgine_Collada_Param();
                geom.Mesh.Source[0].Technique_Common.Accessor.Param[0].Name = "X";
                geom.Mesh.Source[0].Technique_Common.Accessor.Param[0].Type = "float";
                geom.Mesh.Source[0].Technique_Common.Accessor.Param[1] = new Grendgine_Collada_Param();
                geom.Mesh.Source[0].Technique_Common.Accessor.Param[1].Name = "Y";
                geom.Mesh.Source[0].Technique_Common.Accessor.Param[1].Type = "float";
                geom.Mesh.Source[0].Technique_Common.Accessor.Param[2] = new Grendgine_Collada_Param();
                geom.Mesh.Source[0].Technique_Common.Accessor.Param[2].Name = "Z";
                geom.Mesh.Source[0].Technique_Common.Accessor.Param[2].Type = "float";

                geom.Mesh.Vertices = new Grendgine_Collada_Vertices();
                geom.Mesh.Vertices.ID = uniqueID + "-VERTEX";
                geom.Mesh.Vertices.Input = new Grendgine_Collada_Input_Unshared[1];
                geom.Mesh.Vertices.Input[0] = new Grendgine_Collada_Input_Unshared();
                geom.Mesh.Vertices.Input[0].Semantic = Grendgine_Collada_Input_Semantic.POSITION;
                geom.Mesh.Vertices.Input[0].source = "#" + geom.Mesh.Source[0].ID;
                geom.Mesh.Triangles = new Grendgine_Collada_Triangles[1];
                geom.Mesh.Triangles[0] = new Grendgine_Collada_Triangles();
                geom.Mesh.Triangles[0].Count = nF*2;
                geom.Mesh.Triangles[0].Input = new Grendgine_Collada_Input_Shared[1];
                geom.Mesh.Triangles[0].Input[0] = new Grendgine_Collada_Input_Shared();
                geom.Mesh.Triangles[0].Input[0].Semantic = Grendgine_Collada_Input_Semantic.VERTEX;
                geom.Mesh.Triangles[0].Input[0].Offset = 0;
                geom.Mesh.Triangles[0].Input[0].source = "#" + geom.Mesh.Vertices.ID;
                geom.Mesh.Triangles[0].P = new Grendgine_Collada_Int_Array_String();
                geom.Mesh.Triangles[0].P.Value_As_String = "0 1 2 2 1 3 1 5 3 3 5 7 4 0 6 6 0 2 5 4 7 7 4 6 0 4 1 1 4 5 2 3 6 6 3 7";
            }

            col_scenes.Library_Visual_Scene = new Grendgine_Collada_Library_Visual_Scenes();
            col_scenes.Library_Visual_Scene.Visual_Scene = new Grendgine_Collada_Visual_Scene[1];
            col_scenes.Library_Visual_Scene.Visual_Scene[0] = new Grendgine_Collada_Visual_Scene();
            Grendgine_Collada_Visual_Scene scene = col_scenes.Library_Visual_Scene.Visual_Scene[0];
            scene.ID = "test";
            scene.Name = "test";

            scene.Node = new Grendgine_Collada_Node[sticksAll.Count];
            for (int i = 0; i < sticksAll.Count; i++)
            {
                string uniqueID = "Stick" + i.ToString();
                Grendgine_Collada_Geometry geom = col_scenes.Library_Geometries.Geometry[i];
                Matrix T = sticksAll[i].WorldTransform;

                scene.Node[i] = new Grendgine_Collada_Node();
                scene.Node[i].Name = uniqueID;
                scene.Node[i].ID = uniqueID;
                scene.Node[i].sID = uniqueID;
                scene.Node[i].Type = Grendgine_Collada_Node_Type.NODE;
                scene.Node[i].Instance_Geometry = new Grendgine_Collada_Instance_Geometry[1];
                scene.Node[i].Instance_Geometry[0] = new Grendgine_Collada_Instance_Geometry();
                scene.Node[i].Instance_Geometry[0].URL = "#" + geom.ID;
                scene.Node[i].Matrix = new Grendgine_Collada_Matrix[1];
                scene.Node[i].Matrix[0] = new Grendgine_Collada_Matrix();
                scene.Node[i].Matrix[0].sID = "matrix";
                string val = "";
                for (int j = 0; j < 4; j++)
                {
                    for (int k = 0; k < 4; k++)
                    {
                        val += "  "+T[k, j].ToString();
                    }
                }
                scene.Node[i].Matrix[0].Value_As_String = val;

                col_scenes.Scene = new Grendgine_Collada_Scene();
                col_scenes.Scene.Visual_Scene = new Grendgine_Collada_Instance_Visual_Scene();
                col_scenes.Scene.Visual_Scene.URL = "#" + scene.ID;
            }
            XmlSerializer sr = new XmlSerializer(typeof(Grendgine_Collada));
            SaveFileDialog sfd = new SaveFileDialog();
            sfd.FileName = "sticks.dae";
            sfd.Filter = "COLLADA(*.dae)|Any type(*.*)";
            sfd.FilterIndex = 0;
            sfd.Title = "File Open";
            sfd.RestoreDirectory = true;
            if (sfd.ShowDialog() == DialogResult.OK)
            {
                string file_name = sfd.FileName;
                TextWriter tr = new StreamWriter(file_name);
                sr.Serialize(tr, col_scenes);
                tr.Close();
            }
        }
        void dropStickAlongPath(Vector3 pathPt)
        {
            Vector3 pos = pathPt;
            //Vector3 localInertia;
            //box.CalculateLocalInertia(stickMass, out localInertia);
            RigidBody cmbody = LocalCreateRigidBody(stickMass, Matrix.Translation(pos), box);
            cmbody.Friction = friction;
            count++;
            sticks.Add(cmbody);

        }
        //Similar to draw()
        public override void OnUpdate()
        {
            for (int i = 0; i < sticks.Count; i++)
            {
                RigidBody stick = sticks[i];
                Vector3 v = stick.LinearVelocity;
                if (v.LengthSquared() < 0.001)
                {
                    stick.SetMassProps(0, new Vector3(0, 0, 0));
                    stick.UserObject = "static";
                    frozenSticks.Add(stick);
                    sticks.Remove(stick);
                }
            }
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
            cp5 = new ControlPanel(900);
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

            cp5.addSlider("releaseHt", (val) => { releaseHt = (float)val * scale; renewHeight(); }, 500, 2000, 500, 20, 640);

            //cp5.addButton("freeze!", () => { freeze(); }, 20, 690);
            cp5.addButton("Run!", () => { run = !run; }, 20, 740);
            cp5.addButton("Export!", () => { export(); }, 20, 790);
            cp5.Show();
            gravity = 300 * scale;
            friction = 1f;
            //stick
            stickMass = 1 * scale;
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
            box = new BoxShape(stickSizeX/2f, stickSizeZ/2f, stickSizeY/2f);
            stickSample = LocalCreateRigidBody(0, Matrix.Translation(new Vector3(-50,0,-30)), box);
            renewStick();
        }
        void renewHeight()
        {
            if (pos == null) return;
            pathStPt = new Vector3(pathStX, releaseHt, 0);
            pathEndPt = new Vector3(pathEndX, releaseHt, 0);
            Vector3 pathPos = new Vector3(pathStPt.X, pathStPt.Y, pathStPt.Z);
            Vector3 pathDir = new Vector3((pathEndPt.X - pathStPt.X) / pathDiv, (pathEndPt.Y - pathStPt.Y) / pathDiv, (pathEndPt.Z - pathStPt.Z) / pathDiv);
            for (int i = 0; i < pathDiv; i++)
            {
                pathPos += pathDir;
                pos[i] = new Vector3(pathPos.X, pathPos.Y, pathPos.Z);
            }
        }
        void renewStick()
        {
            if (box == null) return;
            box = new BoxShape(stickSizeX/2f, stickSizeZ/2f, stickSizeY/2f);
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
