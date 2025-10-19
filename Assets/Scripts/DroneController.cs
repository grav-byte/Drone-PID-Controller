using UnityEngine;

public class DroneController: MonoBehaviour
{ 
	[Header("References")]
	private Rigidbody _rigidbody;
	public Transform motor1;
	public Transform motor2;
	public Transform motor3;
	public Transform motor4;

	[Header("PID Parameters")]
	public PIDParameters rollTargetParameters;
	public PIDParameters pitchTargetParameters;
	
	public PIDParameters rollParameters;
	public PIDParameters pitchParameters;
	public PIDParameters yawParameters;
	public PIDParameters thrustParameters;

	[Header("User Input")]
	public float maxVelocity;
	public float maxYaw;
	public float maxThrustInput;
	public float maxBankAngle = 45;

	public bool userControl = true;

	[Header("Drone Control")] 
	public float rollPitchForce = 0.15f;
	public float yawForce = 0.15f;
	public float thrustForce = 1.5f;
	
	public float minThrust = 0.1f; // [N] absolute min force motor will always provide
	public float maxThrust = 2.0f; // [N] absolute max force one motor can provide, will be clamped to this value
	
	[Header("Debug")]
	public bool debugEnabled = true;
	public Color motorMaxedColor = Color.red;
	public Color motorMinColor = Color.coral;
	public Color motorNormalColor = Color.green;
	private MeshRenderer[] _renderers;
	
	[Header("Environment")]
	public Vector3 windSpeed;
	
	private float _targetVelocityX;
	private float _targetVelocityY;
	private float _targetVelocityZ;
	public float targetYaw;
	private float _lastVelocityX;
	private float _lastVelocityY;
	private float _lastVelocityZ;

	private PIDController _rollTargetController;
	private PIDController _pitchTargetController;
	private PIDController _rollController;
	private PIDController _pitchController;
	private PIDController _thrustController;
	private PIDController _yawController;

	void Start()
	{
		_rigidbody = GetComponent<Rigidbody>();
		_rollController = new PIDController(rollParameters);
		_thrustController  = new PIDController(thrustParameters);
		_rollTargetController = new PIDController(rollTargetParameters);
		_pitchTargetController = new PIDController(pitchTargetParameters);
		_pitchController = new PIDController(pitchParameters);
		_yawController = new PIDController(yawParameters);
		_renderers = new[]
		{
			motor1.GetComponent<MeshRenderer>(),
			motor2.GetComponent<MeshRenderer>(),
			motor3.GetComponent<MeshRenderer>(),
			motor4.GetComponent<MeshRenderer>()
		};
	}

	private void Update()
	{
		//remote control
		_targetVelocityX = -Input.GetAxis("Horizontal") * maxVelocity;
		_targetVelocityZ = -Input.GetAxis("Vertical") * maxVelocity;
		targetYaw += Input.GetKey(KeyCode.E) ? maxYaw : Input.GetKey(KeyCode.Q) ? -maxYaw : 0;
		_targetVelocityY = Input.GetKey(KeyCode.LeftShift) ? maxThrustInput : Input.GetKey(KeyCode.LeftControl) ? -maxThrustInput : 0;
	}

	private void FixedUpdate()
	{
		var localVelocity = transform.InverseTransformDirection(_rigidbody.linearVelocity);
		var localAngularVelocity = transform.InverseTransformDirection(_rigidbody.angularVelocity) * Mathf.Rad2Deg;
		
		//pitch target
		var zAccel = (_lastVelocityZ - localVelocity.z) / Time.fixedDeltaTime;
		_lastVelocityZ = localVelocity.z;
		var pitchTarget = _pitchTargetController.UpdatePID(Time.fixedDeltaTime, _targetVelocityZ, _lastVelocityZ, zAccel, false);
		pitchTarget *= maxBankAngle;
		
		//pitch
		var targetAngle = userControl ? pitchTarget : 0;
		var pitchCmd = _pitchController.UpdatePIDAngle(Time.fixedDeltaTime, targetAngle, transform.localEulerAngles.x, localAngularVelocity.x);
		pitchCmd *= rollPitchForce;
		
		//yaw
		var yawCmd = _yawController.UpdatePIDAngle(Time.fixedDeltaTime, targetYaw, transform.localEulerAngles.y, localAngularVelocity.y);
		yawCmd *= yawForce;
		
		//roll target
		var xAccel = (_lastVelocityX - localVelocity.x) / Time.fixedDeltaTime;
		_lastVelocityX = localVelocity.x;
		var rollTarget = _rollTargetController.UpdatePID(Time.fixedDeltaTime, _targetVelocityX, _lastVelocityX, xAccel, false);
		rollTarget *= -maxBankAngle;
		
		//roll
		var rollAngle = userControl ? rollTarget : 0;
		var rollCmd = _rollController.UpdatePIDAngle(Time.fixedDeltaTime, rollAngle, transform.localEulerAngles.z, localAngularVelocity.z);
		rollCmd *= rollPitchForce;
		
		//thrust
		var accelerationY = (_lastVelocityY - _rigidbody.linearVelocity.y) / Time.fixedDeltaTime;
		_lastVelocityY = _rigidbody.linearVelocity.y;
		var thrustCmd = _thrustController.UpdatePID(Time.fixedDeltaTime, _targetVelocityY, _rigidbody.linearVelocity.y, accelerationY, false);
		thrustCmd += _rigidbody.mass * 9.81f / 4;
		var motor1Cmd = thrustCmd + rollCmd - pitchCmd - yawCmd;
		var motor2Cmd = thrustCmd + rollCmd + pitchCmd + yawCmd;
		var motor3Cmd = thrustCmd - rollCmd + pitchCmd - yawCmd;
		var motor4Cmd = thrustCmd - rollCmd - pitchCmd + yawCmd;
		
		//clamp motor commands
		motor1Cmd = Mathf.Clamp(motor1Cmd, minThrust, maxThrust);
		motor2Cmd = Mathf.Clamp(motor2Cmd, minThrust, maxThrust);
		motor3Cmd = Mathf.Clamp(motor3Cmd, minThrust, maxThrust);
		motor4Cmd = Mathf.Clamp(motor4Cmd, minThrust, maxThrust);
		ColorMotor(0, motor1Cmd);
		ColorMotor(1, motor2Cmd);
		ColorMotor(2, motor3Cmd);
		ColorMotor(3, motor4Cmd);
		
		//add forces and torques
		_rigidbody.AddForceAtPosition(motor1.up * motor1Cmd, motor1.position);
		_rigidbody.AddForceAtPosition(motor2.up * motor2Cmd, motor2.position);
		_rigidbody.AddForceAtPosition(motor3.up * motor3Cmd, motor3.position);
		_rigidbody.AddForceAtPosition(motor4.up * motor4Cmd, motor4.position);
		var up = transform.up;
		
		_rigidbody.AddTorque(up * motor1Cmd);
		_rigidbody.AddTorque(-up * motor2Cmd);
		_rigidbody.AddTorque(up * motor3Cmd);
		_rigidbody.AddTorque(-up * motor4Cmd);
		
		//add wind
		_rigidbody.AddForce(windSpeed, ForceMode.Acceleration);
	}

	private void ColorMotor(int idx, float cmd)
	{
		if (cmd == minThrust)
			_renderers[idx].material.color = motorMinColor;
		else if (cmd == maxThrust)
			_renderers[idx].material.color = motorMaxedColor;
		else
			_renderers[idx].material.color = motorNormalColor;
	}
}