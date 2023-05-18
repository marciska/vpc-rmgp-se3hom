using UnityEngine;

public class TargetController : MonoBehaviour
{
    // configs
    [Header("General settings")]
    [Tooltip("Bird only starts trajectory if this is set to true")]
    [SerializeField] bool started = false;
    public bool drawGizmos = true;

    [Header("Trajectory oscillator settings")]
    public float epsilon = .25f;
    [Min(0f)]
    public float speed = 1.5f;
    [Min(.1f)]
    public float scale = 1f;
    [Tooltip("Center of the trajectory")]
    public Transform center;

    // states
    Vector3 offset = Vector3.zero;
    public Vector3 vel = Vector3.zero;

    // for integration
    delegate Vector3 velFunc(Vector3 pos);
    Vector3 ode3step(velFunc method, Vector3 pos, float h)
    {
        Vector3 k1 = method(pos);
        Vector3 k2 = method(pos + 1f/2f*h*k1);
        Vector3 k3 = method(pos + 3f/4f*h*k2);
        return pos + 2f/9f*h*k1 + 1f/3f*h*k2 + 4f/9f*h*k3;
    }

    // Start is called before the first frame update
    void Start() => SetTrajectoryCenter();

    private void SetTrajectoryCenter()
    {
        if (center) offset = center.position;
    }

    void FixedUpdate() => Move();

    void Move()
    {
        if (!started) return;

        // do one integration step
        velFunc method = (Vector3 pos) => {return speed*Quartic((pos - offset)/scale, epsilon);};
        Vector3 newpos = ode3step(method, transform.position, Time.deltaTime);

        // update position and orientation
        vel = (newpos - transform.position)/Time.deltaTime;
        transform.position = newpos;
        if (vel.sqrMagnitude != 0f) transform.rotation = Quaternion.LookRotation(vel);
    }

    public Vector3 VanDerPol(Vector3 position, float epsilon)
    {
        Vector3 velocity = Vector3.zero;
        velocity.x = position.z;
        velocity.y = 0;
        velocity.z = -position.x + epsilon * (1 - Mathf.Pow(position.x, 2)) * position.z;
        return velocity;
    }

    public Vector3 Duffing(Vector3 position, float alpha, float beta, float gamma, float delta, float omega)
    {
        Vector3 velocity = Vector3.zero;
        velocity.x = position.z;
        velocity.y = 0;
        velocity.z = -delta * position.z - alpha * position.x - beta * Mathf.Pow(position.x, 3) + gamma * Mathf.Cos(omega * Time.time);
        return velocity;
    }

    public Vector3 Quartic(Vector3 position, float k)
    {
        float theta = Mathf.Atan2(position.z,position.x) - Mathf.PI/4f;

        Vector3 velocity = Vector3.zero;
        velocity.x = position.z;
        velocity.y = Mathf.Cos(theta);
        velocity.z = k * (-Mathf.Pow(position.x, 3) + position.x);
        return velocity;
    }

    public Vector3 Lorenz(Vector3 position, float sigma, float rho, float beta)
    {
        var x = position.x;
        var y = position.z;
        var z = position.y;
        Vector3 velocity = Vector3.zero;
        velocity.x = sigma * (y - x);
        velocity.y = x * (rho - z) - y;
        velocity.z = x * y - beta * z;
        return velocity;
    }

    public void StartTrajectory()
    {
        if (!started)
            started = true;
    }

    void OnDrawGizmosSelected()
    {
        if (!drawGizmos)
            return;

        // Draw lines for trajectories
        const int M = 150;
        const float dt = 0.1f;
        SetTrajectoryCenter();
        Vector3[] positions = new Vector3[M];
        positions[0] = transform.position;
        // positions[0].y += 1f;

        velFunc method = (Vector3 pos) => {return speed*Quartic((pos - offset)/scale, epsilon);};

        // first trajectory
        Gizmos.color = Color.magenta;
        for (int i = 1; i < M; i++)
        {
            positions[i] = ode3step(method, positions[i - 1], dt);
            Gizmos.DrawLine(positions[i - 1], positions[i]);
        }
    }
}
