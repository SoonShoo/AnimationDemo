﻿using System;
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
using SkeletonLib;
using NeuralNetworkTest;
using NeuronDotNet.Core.Backpropagation;
using NeuronDotNet.Core;

namespace PhysicsDemo {
	public class PhysicsDemo: Game {

		/// <summary>
		/// PhysicsDemo constructor
		/// </summary>
		public PhysicsDemo ()
			: base()
		{
			//	enable object tracking :
			Parameters.TrackObjects = false;
			Parameters.VSyncInterval = 0;
			Parameters.MsaaLevel = 4;

			//	add services :
			AddService(new SpriteBatch(this), false, false, 0, 0);
			AddService(new DebugStrings(this), true, true, 9999, 9999);
			AddService(new DebugRender(this), true, true, 9998, 9998);
			AddService(new Camera(this), true, false, 1, 1);

			//	load configuration :
			LoadConfiguration();

			//	Force to enable free camera.
			GetService<Camera>().Config.FreeCamEnabled	=	true;

			//	make configuration saved on exit
			Exiting += FusionGame_Exiting;
			InputDevice.KeyDown += InputDevice_KeyDown;
		}

        Scene scene;
		Space space;
		ConstantBuffer constBuffer;
		Ubershader uberShader;
		StateFactory factory;
		bool flag = false;
		Texture2D texture;
        CubeVertex[] data;
		Random random = new Random();
        BaseSkeleton baseSkeleton;
        BackpropagationNetwork network;

        int currentMotor = 0;
        float yaw = 0, pitch = 0, roll = 0;

        private void createNetwork() {
            network = NeuralNetworkTest.Program.createNetwork();
        }

		struct CBData {
			public Fusion.Mathematics.Matrix Projection;
			public Fusion.Mathematics.Matrix View;
			public Fusion.Mathematics.Matrix World;
			public Vector4Fusion ViewPos;
			public Vector4Fusion Color;
		}


		enum RenderFlags {
			None,
		}


		struct CubeVertex {
			[Vertex("POSITION")]
			public Vector3Fusion Position;
			[Vertex("NORMAL")]
			public Vector3Fusion Normal;
			[Vertex("COLOR")]
			public Vector4Fusion Color;
			[Vertex("TEXCOORD")]
			public Fusion.Mathematics.Vector2 TexCoord;
		}

		VertexBuffer vb;
		IndexBuffer ib;

		/// <summary>
		/// Add services :
		/// </summary>
		protected override void Initialize ()
		{
			base.Initialize();

			constBuffer = new ConstantBuffer(GraphicsDevice, typeof(CBData));

			LoadContent();
			Reloading += (s, e) => LoadContent();

			GetService<Camera>().FreeCamPosition = new Vector3Fusion(0, 12, 21);
						
			//	fill vertex buffer for cube:
			Vector4Fusion color = new Vector4Fusion(Vector3Fusion.Zero, 1);
			Fusion.Mathematics.Vector2 texcoord = Fusion.Mathematics.Vector2.Zero;

			// back face
			var v0 = new CubeVertex { Position = new Vector3Fusion(-0.5f, -0.5f, -0.5f), Normal = new Vector3Fusion(-1.0f, 0, 0), Color = color, TexCoord = texcoord };
			var v1 = new CubeVertex { Position = new Vector3Fusion(-0.5f, -0.5f, 0.5f), Normal = new Vector3Fusion(-1.0f, 0, 0), Color = color, TexCoord = texcoord };
			var v2 = new CubeVertex { Position = new Vector3Fusion(-0.5f, 0.5f, 0.5f), Normal = new Vector3Fusion(-1.0f, 0, 0), Color = color, TexCoord = texcoord };
			var v3 = new CubeVertex { Position = new Vector3Fusion(-0.5f, 0.5f, -0.5f), Normal = new Vector3Fusion(-1.0f, 0, 0), Color = color, TexCoord = texcoord };

			// front
			var v4 = new CubeVertex { Position = new Vector3Fusion(0.5f, -0.5f, -0.5f), Normal = new Vector3Fusion(1.0f, 0, 0), Color = color, TexCoord = texcoord };
			var v5 = new CubeVertex { Position = new Vector3Fusion(0.5f, -0.5f, 0.5f), Normal = new Vector3Fusion(1.0f, 0, 0), Color = color, TexCoord = texcoord };
			var v6 = new CubeVertex { Position = new Vector3Fusion(0.5f, 0.5f, 0.5f), Normal = new Vector3Fusion(1.0f, 0, 0), Color = color, TexCoord = texcoord };
			var v7 = new CubeVertex { Position = new Vector3Fusion(0.5f, 0.5f, -0.5f), Normal = new Vector3Fusion(1.0f, 0, 0), Color = color, TexCoord = texcoord };

			// left
			var v8 = new CubeVertex { Position = new Vector3Fusion(-0.5f, -0.5f, -0.5f), Normal = new Vector3Fusion(0, 0, -1.0f), Color = color, TexCoord = texcoord };
			var v9 = new CubeVertex { Position = new Vector3Fusion(0.5f, -0.5f, -0.5f), Normal = new Vector3Fusion(0, 0, -1.0f), Color = color, TexCoord = texcoord };
			var v10 = new CubeVertex { Position = new Vector3Fusion(0.5f, 0.5f, -0.5f), Normal = new Vector3Fusion(0, 0, -1.0f), Color = color, TexCoord = texcoord };
			var v11 = new CubeVertex { Position = new Vector3Fusion(-0.5f, 0.5f, -0.5f), Normal = new Vector3Fusion(0, 0, -1.0f), Color = color, TexCoord = texcoord };

			// right
			var v12 = new CubeVertex { Position = new Vector3Fusion(-0.5f, -0.5f, 0.5f), Normal = new Vector3Fusion(0, 0, 1.0f), Color = color, TexCoord = texcoord };
			var v13 = new CubeVertex { Position = new Vector3Fusion(0.5f, -0.5f, 0.5f), Normal = new Vector3Fusion(0, 0, 1.0f), Color = color, TexCoord = texcoord };
			var v14 = new CubeVertex { Position = new Vector3Fusion(0.5f, 0.5f, 0.5f), Normal = new Vector3Fusion(0, 0, 1.0f), Color = color, TexCoord = texcoord };
			var v15 = new CubeVertex { Position = new Vector3Fusion(-0.5f, 0.5f, 0.5f), Normal = new Vector3Fusion(0, 0, 1.0f), Color = color, TexCoord = texcoord };

			// top
			var v16 = new CubeVertex { Position = new Vector3Fusion(0.5f, 0.5f, -0.5f), Normal = new Vector3Fusion(0, 1.0f, 0), Color = color, TexCoord = texcoord };
			var v17 = new CubeVertex { Position = new Vector3Fusion(0.5f, 0.5f, 0.5f), Normal = new Vector3Fusion(0, 1.0f, 0), Color = color, TexCoord = texcoord };
			var v18 = new CubeVertex { Position = new Vector3Fusion(-0.5f, 0.5f, 0.5f), Normal = new Vector3Fusion(0, 1.0f, 0), Color = color, TexCoord = texcoord };
			var v19 = new CubeVertex { Position = new Vector3Fusion(-0.5f, 0.5f, -0.5f), Normal = new Vector3Fusion(0, 1.0f, 0), Color = color, TexCoord = texcoord };

			// bottom
			var v20 = new CubeVertex { Position = new Vector3Fusion(0.5f, -0.5f, -0.5f), Normal = new Vector3Fusion(0, -1.0f, 0), Color = color, TexCoord = texcoord };
			var v21 = new CubeVertex { Position = new Vector3Fusion(0.5f, -0.5f, 0.5f), Normal = new Vector3Fusion(0, -1.0f, 0), Color = color, TexCoord = texcoord };
			var v22 = new CubeVertex { Position = new Vector3Fusion(-0.5f, -0.5f, 0.5f), Normal = new Vector3Fusion(0, -1.0f, 0), Color = color, TexCoord = texcoord };
			var v23 = new CubeVertex { Position = new Vector3Fusion(-0.5f, -0.5f, -0.5f), Normal = new Vector3Fusion(0, -1.0f, 0), Color = color, TexCoord = texcoord };

			data = new CubeVertex[] { v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, v22, v23 };
			vb.SetData(data, 0, 24);

			// fill the index buffer
			var index = new int[] { 3, 1, 0,	// back
									3, 2, 1,
									4, 5, 7,	// front
									7, 5, 6,
									8, 9, 11,	// left
									11, 9, 10,
									14, 13, 12,	// right
									14, 12, 15,
									19, 16, 17,	// top
									19, 17, 18,
									20, 23, 22,	// bottom
									20, 22, 21};
			ib.SetData(index);

            createNetwork();
            /*string[] lines = System.IO.File.ReadAllLines(@"D:\network1.txt");
            double[] weights = NeuralNetworkTest.Program.stringToWeights(lines[0]);
            NeuralNetworkTest.Program.setNetworkWeights(network, weights);
            double[] noutput = network.Run(new double[]{1, 1, 0});
            String temp = "";
            foreach (double d in noutput) {
                temp += d + ", ";
            }
            Console.WriteLine(temp);*/
		}



		/// <summary>
		/// Load content
		/// </summary>
		public void LoadContent ()
		{
			uberShader = Content.Load<Ubershader>("render");
            factory = new StateFactory(uberShader, typeof(RenderFlags), VertexInputElement.FromStructure<CubeVertex>());
			texture = Content.Load<Texture2D>(@"Scenes\lena");
            //scene = Content.Load<Scene>(@"simple4_not_par");

			vb = new VertexBuffer(GraphicsDevice, typeof(CubeVertex), 24);
			ib = new IndexBuffer(GraphicsDevice, 36);

			// create a new space with physics
			space = new Space();
            baseSkeleton = new BaseSkeleton(space);

            flag = true;
		}

		/// <summary>
		/// 
		/// </summary>
		/// <param name="disposing"></param>
		protected override void Dispose (bool disposing)
		{
			if ( disposing ) {
				SafeDispose(ref constBuffer);
				SafeDispose(ref vb);
				SafeDispose(ref ib);
			}

			base.Dispose(disposing);
		}



		/// <summary>
		/// Handle keys for each demo
		/// </summary>
		/// <param name="sender"></param>
		/// <param name="e"></param>
		void InputDevice_KeyDown (object sender, Fusion.Input.InputDevice.KeyEventArgs e)
		{
			if ( e.Key == Keys.F1 ) {
				DevCon.Show(this);
			}

			if ( e.Key == Keys.F2 ) {
				Parameters.ToggleVSync();
			}

			if ( e.Key == Keys.F5 ) {
				Reload();
			}

			if ( e.Key == Keys.F12 ) {
				GraphicsDevice.Screenshot();
			}

			if ( e.Key == Keys.Escape ) {
				Exit();
			}

			// pause physics
			if ( e.Key == Keys.P ) {
				if ( flag ) {
					flag = false;
				} else {
					flag = true;
				}
			}

            if (e.Key == Keys.G)
            {
                yaw += 0.1f;
            }

            if (e.Key == Keys.H)
            {
                pitch += 0.1f;
            }

            if (e.Key == Keys.J)
            {
                roll += 0.1f;
            }


            if (e.Key == Keys.T)
            {
                yaw -= 0.1f;
            }
            if (e.Key == Keys.Y)
            {
                pitch -= 0.1f;
            }

            if (e.Key == Keys.U)
            {
                roll -= 0.1f;
            }

            if (e.Key == Keys.O)
            {
                currentMotor = (currentMotor + 1) % baseSkeleton.angularMotors.Count();
            }
            if (e.Key == Keys.P)
            {
                currentMotor = (currentMotor - 1 + baseSkeleton.angularMotors.Count()) % baseSkeleton.angularMotors.Count();
            }
		}



		/// <summary>
		/// Save configuration on exit.
		/// </summary>
		/// <param name="sender"></param>
		/// <param name="e"></param>
		void FusionGame_Exiting (object sender, EventArgs e)
		{
			SaveConfiguration();
		}

        double time = 0;

		/// <summary>
		/// 
		/// </summary>
		/// <param name="gameTime"></param>
		protected override void Update (GameTime gameTime)
		{
			var ds = GetService<DebugStrings>();

			ds.Add(Color.Orange, "FPS {0}", gameTime.Fps);
			ds.Add("F1   - show developer console");
			ds.Add("F2   - toggle vsync");
			ds.Add("F5   - build content and reload textures");
			ds.Add("F12  - make screenshot");
			ds.Add("ESC  - exit");

			var cam = GetService<Camera>();
			var dr = GetService<DebugRender>();
			dr.View = cam.GetViewMatrix(StereoEye.Mono);
			dr.Projection = cam.GetProjectionMatrix(StereoEye.Mono);

			dr.DrawGrid(10);
            
            time += gameTime.ElapsedSec;
            //System.Console.WriteLine(time / NeuralNetworkTest.Program.timeMax);
            //double[] output = network.Run(new double[] { time / NeuralNetworkTest.Program.timeMax , 1});
            //double[] output = network.Run(new double[] { time / NeuralNetworkTest.Program.timeMax, 1 });
            double[] input = baseSkeleton.getModelInput(time / NeuralNetworkTest.Program.timeMax);
            double[] output = network.Run(input);
            baseSkeleton.setMotorInput(output);

            if (time > NeuralNetworkTest.Program.timeMax)
            {
                space = new Space();
                baseSkeleton = new BaseSkeleton(space);
                time = 0;
            }

            //AngularMotor motor = baseSkeleton.angularMotors[currentMotor];
            //motor.Settings.Mode = MotorMode.Servomechanism;
            //motor.Settings.Servo.Goal = Quaternion.CreateFromYawPitchRoll(yaw, pitch, roll);
			
            // physics updates here
			if ( flag ) {
				space.Update(gameTime.ElapsedSec);
			}

			base.Update(gameTime);
		}




		/// <summary>
		/// 
		/// </summary>
		/// <param name="gameTime"></param>
		/// <param name="stereoEye"></param>
		protected override void Draw (GameTime gameTime, StereoEye stereoEye)
		{
			CBData cbData = new CBData();

			var cam = GetService<Camera>();

			GraphicsDevice.ClearBackbuffer(Color.CornflowerBlue, 1, 0);

			foreach ( var e in space.Entities ) {
				Box box = e as Box;
				if ( box != null ) // this won't create any graphics for an entity that isn't a box
				{
					if ( box.IsDynamic ) // draw only dynamic boxes
					{
						// fill world matrix
						Fusion.Mathematics.Matrix matrix = new Fusion.Mathematics.Matrix(box.WorldTransform.M11, box.WorldTransform.M12, box.WorldTransform.M13, box.WorldTransform.M14,
																									box.WorldTransform.M21, box.WorldTransform.M22, box.WorldTransform.M23, box.WorldTransform.M24,
																									box.WorldTransform.M31, box.WorldTransform.M32, box.WorldTransform.M33, box.WorldTransform.M34,
																									box.WorldTransform.M41, box.WorldTransform.M42, box.WorldTransform.M43, box.WorldTransform.M44);
						cbData.Projection = cam.GetProjectionMatrix(stereoEye);
						cbData.View = cam.GetViewMatrix(stereoEye);
						cbData.World = matrix;
						cbData.ViewPos = new Vector4Fusion(cam.GetCameraMatrix(stereoEye).TranslationVector, 1);
						Color c = new Color(100, 200, 150);
						cbData.Color =  c.ToVector4();

						constBuffer.SetData(cbData);
						
						GraphicsDevice.PipelineState = factory[0];

						GraphicsDevice.PixelShaderConstants[0] = constBuffer;
						GraphicsDevice.VertexShaderConstants[0] = constBuffer;
						GraphicsDevice.PixelShaderSamplers[0] = SamplerState.AnisotropicWrap;
						GraphicsDevice.PixelShaderResources[0] = texture;

						// setup data and draw box
						GraphicsDevice.SetupVertexInput(vb, ib);
						GraphicsDevice.DrawIndexed( Primitive.TriangleList, 36, 0, 0);
					}
				}
			}

			base.Draw(gameTime, stereoEye);
		}
	}
}
