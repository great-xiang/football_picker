using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading;

public class RandomMove : MonoBehaviour
{
    public float MoveSpeed = 1f;

    void FixedUpdate(){
        transform.Translate(Vector3.forward *MoveSpeed*0.1f); 
        }
    private void OnCollisionEnter(Collision collision){
        var ran = new System.Random();
        int angle = ran.Next(90,270);
        transform.Rotate(Vector3.up,angle); 
        }

}



