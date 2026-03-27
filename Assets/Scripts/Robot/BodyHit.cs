using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BodyHit : MonoBehaviour
{
    RobotBalanceAgent agent;

    private void Awake()
    {
        agent = GetComponentInParent<RobotBalanceAgent>();
    }
    private void OnCollisionStay(Collision collision)
    {
        if (collision.gameObject.layer == 3)
        {
            agent.BodyHit(name);
        }
    }
}
