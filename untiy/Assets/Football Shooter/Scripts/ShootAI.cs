using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using Random = UnityEngine.Random;

public enum Direction
{
	Left = 0,
	Right,
	Both
}

public class ShootAI : Shoot
{


	public static Action<float> EventChangeCurveRange = delegate {
	};

	private List<Vector3> _ballPath;
	public int _index;
	private bool _isShootByAI = false;
	//	private MyMultiSelection _multiSelection;
	public float _remainTimeToNextPointZ;
	private float yVelocityTarget;
	private float angleZY;
	private bool _isDown;

	public bool _isTestShootAI = false;


	public float _maxLeft = 0f;
	public float _maxRight = 0f;
	
	
	
	/*************** Debug ****************/
	public float _curveLevel = 0;
	public float _difficulty = 0.5f;
	public bool willBeShootByUser = true;
	public Direction _shootDirection = Direction.Left;

	void OnDrawGizmos ()
	{
		if (_ballPath != null && _ballPath.Count > 0) {
		}
	}
	
	/*******************************/

	protected override void Awake ()
	{
		base.Awake ();

	}

	protected override void Start ()
	{
		base.Start ();

	}

	protected override void Update ()
	{

		if (willBeShootByUser) {
			base.Update ();
		}

	}

	public override void goalEvent (bool isGoal)
	{
		base.goalEvent (isGoal);
	}

	public void shoot ()
	{
		shoot (_shootDirection, _curveLevel, _difficulty);
	}

	public void shoot (Direction shootDirection, float curveLevel, float difficulty)
	{
//		Debug.Log("AI shoot");
		//EventShoot();

		//_isShootByAI = true;

		_ballPath.Clear ();

		_index = _ballPath.Count - 1;			// ball going back from the end of the path 


	}

	public float yEnd = 2.36f;
	public float yMiddle = 2.8f;
	public float xEnd = 3.5f;

	public float yExtra = 0.5f;
	public int slideTest = 6;

	public AnimationCurve _animationCurve;

	void OnCollisionEnter (Collision col)
	{
		if (col.gameObject.tag == "Finish") {
			Destroy (col.gameObject);
		}
	}

	public override void reset (float x, float z)
	{
		base.reset (x, z);
		_isShootByAI = false;
	}

	public override void reset ()
	{
		base.reset ();
		_isShootByAI = false;
	}



	[ System.Serializable]
	public class DataShootAI
	{
		public float _distance;

		public float _yMid_Min;
		public float _yEnd_Min_When_Mid_Min;
		public float _yEnd_Max_When_Mid_Min;

		public float _yMid_Max;
		public float _yEnd_Min_When_Mid_Max;
		public float _yEnd_Max_When_Mid_Max;

		public DataShootAI (float distance, float yMid_Min, float yEnd_Min_When_Mid_Min, float yEnd_Max_When_Mid_Min, float yMid_Max, float yEnd_Min_When_Mid_Max, float yEnd_Max_When_Mid_Max)
		{

		}
	}


}
