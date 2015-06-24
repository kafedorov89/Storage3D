using UnityEngine;
using System.Collections;

public class NewBehaviourScript : MonoBehaviour {

	public GameObject Obj;
	// Use this for initialization
	void Start() {
		GameObject plane = GameObject.CreatePrimitive(PrimitiveType.Plane);
		GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
		cube.transform.position = new Vector3(0, 0.5F, 0);
		GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
		sphere.transform.position = new Vector3(0, 1.5F, 0);
		GameObject capsule = GameObject.CreatePrimitive(PrimitiveType.Capsule);
		capsule.transform.position = new Vector3(2, 1, 0);
		GameObject cylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
		cylinder.transform.position = new Vector3(-2, 1, 0);

		Obj = GameObject.Find("MainCameraCube");
		MeshRenderer m = Obj.GetComponent<MeshRenderer>();
		m.enabled = false;
	}
	
	// Update is called once per frame
	void Update () {

	}
}
