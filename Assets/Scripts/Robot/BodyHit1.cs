using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BodyHit1 : MonoBehaviour
{
    public RobotWalkerCustom agent;


    private void OnCollisionStay(Collision collision)
    {
        agent.BodyHit(gameObject, collision);
    }
}
