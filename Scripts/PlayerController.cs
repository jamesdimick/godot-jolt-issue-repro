using Godot;

namespace MRE
{
	public partial class PlayerController : RigidBody3D
	{
		private Node3D CameraWrapper;
		private Node3D CameraOffset;
		private Node3D CameraGimbalY;
		private Node3D CameraGimbalX;
		private Node3D CameraGimbalZ;
		private Camera3D Camera;
		private Vector2 LookInput;

		public override void _Ready()
		{
			CameraWrapper = GetParent().GetNode<Node3D>("PlayerCamera");
			CameraOffset = CameraWrapper.GetNode<Node3D>("Offset");
			CameraGimbalY = CameraOffset.GetNode<Node3D>("GimbalY");
			CameraGimbalX = CameraGimbalY.GetNode<Node3D>("GimbalX");
			CameraGimbalZ = CameraGimbalX.GetNode<Node3D>("GimbalZ");
			Camera = CameraGimbalZ.GetNode<Camera3D>("Camera");

			Input.MouseMode = Input.MouseModeEnum.Captured;
		}

		public override void _UnhandledInput(InputEvent @event)
		{
			LookInput = @event is InputEventMouseMotion eventMouseMotion ? eventMouseMotion.Relative * 0.1f : Vector2.Zero;
		}

		public override void _Process(double delta)
		{
			if (!CameraWrapper.GlobalPosition.IsEqualApprox(GlobalPosition))
			{
				CameraWrapper.GlobalPosition = GlobalPosition;
			}

			Basis gimblYRot = new(Vector3.Up, Mathf.DegToRad(-LookInput.X));
			Basis gimblXRot = CameraGimbalX.Transform.Basis * new Basis(Vector3.Left, Mathf.DegToRad(LookInput.Y));

			LookInput = Vector2.Zero;

			gimblYRot = CameraGimbalY.GlobalTransform.Basis * gimblYRot;

			if (!gimblYRot.IsEqualApprox(CameraGimbalY.GlobalTransform.Basis))
			{
				CameraGimbalY.GlobalTransform = new Transform3D(gimblYRot, CameraGimbalY.GlobalPosition);
			}

			if (!gimblXRot.IsEqualApprox(CameraGimbalX.Transform.Basis))
			{
				CameraGimbalX.Transform = new Transform3D(gimblXRot, Vector3.Zero);
			}
		}

		public override void _IntegrateForces(PhysicsDirectBodyState3D state)
		{
			float inputV = Input.GetAxis("gameplay_backward", "gameplay_forward");
			float inputH = Input.GetAxis("gameplay_right", "gameplay_left");

			Vector3 camFwd = new(-Camera.GlobalTransform.Basis.Z.X, 0f, -Camera.GlobalTransform.Basis.Z.Z);
			Vector3 camRght = new(-Camera.GlobalTransform.Basis.X.X, 0f, -Camera.GlobalTransform.Basis.X.Z);

			Vector3 direction = (camFwd.Normalized() * inputV) + (camRght.Normalized() * inputH);
			direction = direction.Normalized();

			Vector3 newVelocity = direction * 2000f;
			newVelocity.Y = 0f;

			state.ApplyCentralForce(newVelocity);
		}
	}
}
