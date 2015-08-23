package org.iforce2d;

import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.*;
import com.badlogic.gdx.physics.box2d.joints.*;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.ObjectMap;
import com.badlogic.gdx.utils.ObjectSet;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;
import java.util.Vector;

/*
 Author: Chris Campbell - www.iforce2d.net

 This software is provided 'as-is', without any express or implied
 warranty.  In no event will the authors be held liable for any damages
 arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it
 freely, subject to the following restrictions:
 1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */
public class Jb2dJson {

	public World getWorld() {
		return world;
	}

	private World world;

	public Array<Body> getBodies() {
		return m_bodies;
	}

	public class Jb2dJsonCustomProperties {

		ObjectMap<String, Integer> m_customPropertyMap_int;
		ObjectMap<String, Double> m_customPropertyMap_float;
		ObjectMap<String, String> m_customPropertyMap_string;
		ObjectMap<String, Vector2> m_customPropertyMap_vec2;
		ObjectMap<String, Boolean> m_customPropertyMap_bool;

		public Jb2dJsonCustomProperties() {
			m_customPropertyMap_int = new ObjectMap<String, Integer>();
			m_customPropertyMap_float = new ObjectMap<String, Double>();
			m_customPropertyMap_string = new ObjectMap<String, String>();
			m_customPropertyMap_vec2 = new ObjectMap<String, Vector2>();
			m_customPropertyMap_bool = new ObjectMap<String, Boolean>();
		}

	}

	protected boolean m_useHumanReadableFloats;

	protected int m_simulationPositionIterations;
	protected int m_simulationVelocityIterations;
	protected float m_simulationFPS;

	protected ObjectMap<Integer, Body> m_indexToBodyMap;
	protected ObjectMap<Body, Integer> m_bodyToIndexMap;
	protected ObjectMap<Joint, Integer> m_jointToIndexMap;
	protected Array<Body> m_bodies;
	protected Array<Joint> m_joints;
	protected Array<Jb2dJsonImage> m_images;

	protected ObjectMap<Body, String> m_bodyToNameMap;
	protected ObjectMap<Fixture, String> m_fixtureToNameMap;
	protected ObjectMap<Joint, String> m_jointToNameMap;
	protected ObjectMap<Jb2dJsonImage, String> m_imageToNameMap;

	// This maps an item (Body, Fixture etc) to a set of custom properties.
	// Use null for world properties.
	protected ObjectMap<Object, Jb2dJsonCustomProperties> m_customPropertiesMap;

	protected ObjectSet<Body> m_bodiesWithCustomProperties;
	protected ObjectSet<Fixture> m_fixturesWithCustomProperties;
	protected ObjectSet<Joint> m_jointsWithCustomProperties;
	protected ObjectSet<Jb2dJsonImage> m_imagesWithCustomProperties;
	protected ObjectSet<World> m_worldsWithCustomProperties;

	private static final int maxPolygonVertices = 8;

	public Jb2dJson() {
		this(true);
	}

	public Jb2dJson(boolean useHumanReadableFloats) {

		if (!useHumanReadableFloats) {
			// The floatToHex function is not giving the same results
			// as the original C++ version... not critical so worry about it
			// later.
			System.out.println("Non human readable floats are not implemented yet");
			useHumanReadableFloats = true;
		}

		m_useHumanReadableFloats = useHumanReadableFloats;
		m_simulationPositionIterations = 3;
		m_simulationVelocityIterations = 8;
		m_simulationFPS = 60;

		m_indexToBodyMap = new ObjectMap<Integer, Body>();
		m_bodyToIndexMap = new ObjectMap<Body, Integer>();
		m_jointToIndexMap = new ObjectMap<Joint, Integer>();
		m_bodies = new Array<Body>();
		m_joints = new Array<Joint>();
		m_images = new Array<Jb2dJsonImage>();

		m_bodyToNameMap = new ObjectMap<Body, String>();
		m_fixtureToNameMap = new ObjectMap<Fixture, String>();
		m_jointToNameMap = new ObjectMap<Joint, String>();
		m_imageToNameMap = new ObjectMap<Jb2dJsonImage, String>();

		m_customPropertiesMap = new ObjectMap<Object, Jb2dJsonCustomProperties>();

		m_bodiesWithCustomProperties = new ObjectSet<Body>();
		m_fixturesWithCustomProperties = new ObjectSet<Fixture>();
		m_jointsWithCustomProperties = new ObjectSet<Joint>();
		m_imagesWithCustomProperties = new ObjectSet<Jb2dJsonImage>();
		m_worldsWithCustomProperties = new ObjectSet<World>();
	}

	public JSONObject writeToValue(World world) throws JSONException {
		if (null == world)
			return new JSONObject();

		return b2j(world);
	}

	public String worldToString(World world, int indentFactor) throws JSONException {
		if (null == world)
			return new String();

		return b2j(world).toString(indentFactor);
	}

	public boolean writeToFile(World world, String filename, int indentFactor, StringBuilder errorMsg) {
		if (null == world || null == filename)
			return false;

		PrintWriter writer;
		try {
			writer = new PrintWriter(filename);
		} catch (FileNotFoundException e) {
			errorMsg.append("Could not open file " + filename + "for writing");
			return false;
		}

		try {
			writer.println(b2j(world).toString(indentFactor));
		} catch (JSONException e) {
			errorMsg.append("Error writing JSON to file: " + filename);
		}
		writer.close();

		return true;
	}

	public JSONObject b2j(World world) throws JSONException {
		JSONObject worldValue = new JSONObject();

		m_bodyToIndexMap.clear();
		m_jointToIndexMap.clear();

		vecToJson("gravity", world.getGravity(), worldValue);
/*
		worldValue.put("allowSleep", world.isAllowSleep());
*/
		worldValue.put("autoClearForces", world.getAutoClearForces());
/*
		worldValue.put("warmStarting", world.isWarmStarting());
		worldValue.put("continuousPhysics", world.isContinuousPhysics());
*/
		// worldValue.put("subStepping", world.isSubStepping());
		// worldValue["hasDestructionListener"] =
		// world->HasDestructionListener();
		// worldValue["hasContactFilter"] = world->HasContactFilter();
		// worldValue["hasContactListener"] = world->HasContactListener();

		JSONArray customPropertyValue = writeCustomPropertiesToJson(null);
		if (customPropertyValue.length() > 0)
			worldValue.put("customProperties", customPropertyValue);

		int i = 0;
		Array<Body> bodies = new Array<Body>();
		world.getBodies(bodies);
		for (Body body : bodies) {
			m_bodyToIndexMap.put(body, i);
			worldValue.append("body", b2j(body));
			i++;
		}

		// need two passes for joints because gear joints reference other joints
		i = 0;
		Array<Joint> joints = new Array<Joint>();
		world.getJoints(joints);
		for (Joint joint : joints) {
			if (joint.getType() == JointDef.JointType.GearJoint)
				continue;
			m_jointToIndexMap.put(joint, i);
			worldValue.append("joint", b2j(joint));
			i++;
		}
		for (Joint joint : joints) {
			if (joint.getType() != JointDef.JointType.GearJoint)
				continue;
			m_jointToIndexMap.put(joint, i);
			worldValue.append("joint", b2j(joint));
			i++;
		}

		// Currently the only efficient way to add images to a Jb2dJson
		// is by using the R.U.B.E editor. This code has not been tested,
		// but should work ok.
		i = 0;
		for (Jb2dJsonImage image : m_imageToNameMap.keys()) {
			worldValue.append("image", b2j(image));
		}

		m_bodyToIndexMap.clear();
		m_jointToIndexMap.clear();

		return worldValue;
	}

	public void setBodyName(Body body, String name) {
		m_bodyToNameMap.put(body, name);
	}

	public void setFixtureName(Fixture fixture, String name) {
		m_fixtureToNameMap.put(fixture, name);
	}

	public void setJointName(Joint joint, String name) {
		m_jointToNameMap.put(joint, name);
	}

	public void setImageName(Jb2dJsonImage image, String name) {
		m_imageToNameMap.put(image, name);
	}

	public JSONObject b2j(Body body) throws JSONException {
		JSONObject bodyValue = new JSONObject();

		String bodyName = getBodyName(body);
		if (null != bodyName)
			bodyValue.put("name", bodyName);

		switch (body.getType()) {
			case StaticBody:
				bodyValue.put("type", 0);
				break;
			case KinematicBody:
				bodyValue.put("type", 1);
				break;
			case DynamicBody:
				bodyValue.put("type", 2);
				break;
		}

		vecToJson("position", body.getPosition(), bodyValue);
		floatToJson("angle", body.getAngle(), bodyValue);

		vecToJson("linearVelocity", body.getLinearVelocity(), bodyValue);
		floatToJson("angularVelocity", body.getAngularVelocity(), bodyValue);

		if (body.getLinearDamping() != 0)
			floatToJson("linearDamping", body.getLinearDamping(), bodyValue);
		if (body.getAngularDamping() != 0)
			floatToJson("angularDamping", body.getAngularDamping(), bodyValue);
		if (body.getGravityScale() != 1)
			floatToJson("gravityScale", body.getGravityScale(), bodyValue);

		if (body.isBullet())
			bodyValue.put("bullet", true);
		if (!body.isSleepingAllowed())
			bodyValue.put("allowSleep", false);
		if (body.isAwake())
			bodyValue.put("awake", true);
		if (!body.isActive())
			bodyValue.put("active", false);
		if (body.isFixedRotation())
			bodyValue.put("fixedRotation", true);

		MassData massData = body.getMassData();
		if (massData.mass != 0)
			floatToJson("massData-mass", massData.mass, bodyValue);
		if (massData.center.x != 0 || massData.center.y != 0)
			vecToJson("massData-center", massData.center, bodyValue);
		if (massData.I != 0) {
			floatToJson("massData-I", massData.I, bodyValue);
		}

		int i = 0;
		for (Fixture fixture : body.getFixtureList())
			bodyValue.append("fixture", b2j(fixture));

		JSONArray customPropertyValue = writeCustomPropertiesToJson(body);
		if (customPropertyValue.length() > 0)
			bodyValue.put("customProperties", customPropertyValue);

		return bodyValue;
	}

	public JSONObject b2j(Fixture fixture) throws JSONException {
		JSONObject fixtureValue = new JSONObject();

		String fixtureName = getFixtureName(fixture);
		if (null != fixtureName)
			fixtureValue.put("name", fixtureName);

		if (fixture.getRestitution() != 0)
			floatToJson("restitution", fixture.getRestitution(), fixtureValue);
		if (fixture.getFriction() != 0)
			floatToJson("friction", fixture.getFriction(), fixtureValue);
		if (fixture.getDensity() != 0)
			floatToJson("density", fixture.getDensity(), fixtureValue);
		if (fixture.isSensor())
			fixtureValue.put("sensor", true);

		Filter filter = fixture.getFilterData();
		if (filter.categoryBits != 0x0001)
			fixtureValue.put("filter-categoryBits", filter.categoryBits);
		if (filter.maskBits != 0xffff)
			fixtureValue.put("filter-maskBits", filter.maskBits);
		if (filter.groupIndex != 0)
			fixtureValue.put("filter-groupIndex", filter.groupIndex);

		Vector2 tmp = new Vector2();

		Shape shape = fixture.getShape();
		switch (shape.getType()) {
			case Circle: {
				CircleShape circle = (CircleShape) shape;
				JSONObject shapeValue = new JSONObject();
				floatToJson("radius", circle.getRadius(), shapeValue);
				vecToJson("center", circle.getPosition(), shapeValue);
				fixtureValue.put("circle", shapeValue);
			}
			break;
			case Edge: {
				EdgeShape edge = (EdgeShape) shape;
				JSONObject shapeValue = new JSONObject();
				edge.getVertex1(tmp);
				vecToJson("vertex1", tmp, shapeValue);
				edge.getVertex2(tmp);
				vecToJson("vertex2", tmp, shapeValue);
				if (edge.hasVertex0())
					shapeValue.put("hasVertex0", true);
				if (edge.hasVertex3())
					shapeValue.put("hasVertex3", true);
				if (edge.hasVertex0())
					edge.getVertex0(tmp);
				vecToJson("vertex0", tmp, shapeValue);
				if (edge.hasVertex3())
					edge.getVertex3(tmp);
				vecToJson("vertex3", tmp, shapeValue);
				fixtureValue.put("edge", shapeValue);
			}
			break;
			case Chain: {
				ChainShape chain = (ChainShape) shape;
				JSONObject shapeValue = new JSONObject();
				int count = chain.getVertexCount();
				for (int i = 0; i < count; ++i) {
					chain.getVertex(i, tmp);
					vecToJson("vertices", tmp, shapeValue, i);
				}
/*
				if (chain.m_hasPrevVertex)
					shapeValue.put("hasPrevVertex", true);
				if (chain.m_hasNextVertex)
					shapeValue.put("hasNextVertex", true);
				if (chain.m_hasPrevVertex)
					vecToJson("prevVertex", chain.m_prevVertex, shapeValue);
				if (chain.m_hasNextVertex)
					vecToJson("nextVertex", chain.m_nextVertex, shapeValue);
*/
				fixtureValue.put("chain", shapeValue);
			}
			break;
			case Polygon: {
				PolygonShape poly = (PolygonShape) shape;
				JSONObject shapeValue = new JSONObject();
				int vertexCount = poly.getVertexCount();
				for (int i = 0; i < vertexCount; ++i) {
					poly.getVertex(i, tmp);
					vecToJson("vertices", tmp, shapeValue, i);
				}
				fixtureValue.put("polygon", shapeValue);
			}
			break;
			default:
				System.out.println("Unknown shape type : " + shape.getType());
		}

		JSONArray customPropertyValue = writeCustomPropertiesToJson(fixture);
		if (customPropertyValue.length() > 0)
			fixtureValue.put("customProperties", customPropertyValue);

		return fixtureValue;
	}

	public JSONObject b2j(Joint joint) throws JSONException {
		JSONObject jointValue = new JSONObject();

		String jointName = getJointName(joint);
		if (null != jointName)
			jointValue.put("name", jointName);

		int bodyIndexA = lookupBodyIndex(joint.getBodyA());
		int bodyIndexB = lookupBodyIndex(joint.getBodyB());
		jointValue.put("bodyA", bodyIndexA);
		jointValue.put("bodyB", bodyIndexB);
		if (joint.getCollideConnected())
			jointValue.put("collideConnected", true);

		Body bodyA = joint.getBodyA();
		Body bodyB = joint.getBodyB();

		// why do Joint.getAnchor methods need to take an argOut style
		// parameter!?
		Vector2 tmpAnchor = new Vector2();

		switch (joint.getType()) {
			case RevoluteJoint: {
				jointValue.put("type", "revolute");

				RevoluteJoint revoluteJoint = (RevoluteJoint) joint;
				tmpAnchor = revoluteJoint.getAnchorA();
				vecToJson("anchorA", bodyA.getLocalPoint(tmpAnchor), jointValue);
				tmpAnchor = revoluteJoint.getAnchorB();
				vecToJson("anchorB", bodyB.getLocalPoint(tmpAnchor), jointValue);
				floatToJson("refAngle", bodyB.getAngle() - bodyA.getAngle() - revoluteJoint.getJointAngle(), jointValue);
				floatToJson("jointSpeed", revoluteJoint.getJointSpeed(), jointValue);
				jointValue.put("enableLimit", revoluteJoint.isLimitEnabled());
				floatToJson("lowerLimit", revoluteJoint.getLowerLimit(), jointValue);
				floatToJson("upperLimit", revoluteJoint.getUpperLimit(), jointValue);
				jointValue.put("enableMotor", revoluteJoint.isMotorEnabled());
				floatToJson("motorSpeed", revoluteJoint.getMotorSpeed(), jointValue);
				floatToJson("maxMotorTorque", revoluteJoint.getMaxMotorTorque(), jointValue);
			}
			break;
			case PrismaticJoint: {
				jointValue.put("type", "prismatic");

				PrismaticJoint prismaticJoint = (PrismaticJoint) joint;
				tmpAnchor = prismaticJoint.getAnchorA();
				vecToJson("anchorA", bodyA.getLocalPoint(tmpAnchor), jointValue);
				tmpAnchor = prismaticJoint.getAnchorB();
				vecToJson("anchorB", bodyB.getLocalPoint(tmpAnchor), jointValue);
				vecToJson("localAxisA", prismaticJoint.getLocalAxisA(), jointValue);
				floatToJson("refAngle", prismaticJoint.getReferenceAngle(), jointValue);
				jointValue.put("enableLimit", prismaticJoint.isLimitEnabled());
				floatToJson("lowerLimit", prismaticJoint.getLowerLimit(), jointValue);
				floatToJson("upperLimit", prismaticJoint.getUpperLimit(), jointValue);
				jointValue.put("enableMotor", prismaticJoint.isMotorEnabled());
				floatToJson("maxMotorForce", prismaticJoint.getMaxMotorForce(), jointValue);
				floatToJson("motorSpeed", prismaticJoint.getMotorSpeed(), jointValue);
			}
			break;
			case DistanceJoint: {
				jointValue.put("type", "distance");

				DistanceJoint distanceJoint = (DistanceJoint) joint;
				tmpAnchor = distanceJoint.getAnchorA();
				vecToJson("anchorA", bodyA.getLocalPoint(tmpAnchor), jointValue);
				tmpAnchor = distanceJoint.getAnchorB();
				vecToJson("anchorB", bodyB.getLocalPoint(tmpAnchor), jointValue);
				floatToJson("length", distanceJoint.getLength(), jointValue);
				floatToJson("frequency", distanceJoint.getFrequency(), jointValue);
				floatToJson("dampingRatio", distanceJoint.getDampingRatio(), jointValue);
			}
			break;
			case PulleyJoint: {
				jointValue.put("type", "pulley");

				PulleyJoint pulleyJoint = (PulleyJoint) joint;
				vecToJson("groundAnchorA", pulleyJoint.getGroundAnchorA(), jointValue);
				vecToJson("groundAnchorB", pulleyJoint.getGroundAnchorB(), jointValue);
				tmpAnchor = pulleyJoint.getAnchorA();
				vecToJson("anchorA", bodyA.getLocalPoint(tmpAnchor), jointValue);
				floatToJson("lengthA", (pulleyJoint.getGroundAnchorA().sub(tmpAnchor)).len(), jointValue);
				tmpAnchor = pulleyJoint.getAnchorB();
				vecToJson("anchorB", bodyB.getLocalPoint(tmpAnchor), jointValue);
				floatToJson("lengthB", (pulleyJoint.getGroundAnchorB().sub(tmpAnchor)).len(), jointValue);
				floatToJson("ratio", pulleyJoint.getRatio(), jointValue);
			}
			break;
			case MouseJoint: {
				jointValue.put("type", "mouse");

				MouseJoint mouseJoint = (MouseJoint) joint;
				vecToJson("target", mouseJoint.getTarget(), jointValue);
				tmpAnchor = mouseJoint.getAnchorB();
				vecToJson("anchorB", tmpAnchor, jointValue);
				floatToJson("maxForce", mouseJoint.getMaxForce(), jointValue);
				floatToJson("frequency", mouseJoint.getFrequency(), jointValue);
				floatToJson("dampingRatio", mouseJoint.getDampingRatio(), jointValue);
			}
			break;
			case GearJoint: {
				// Wheel joints are apparently not implemented in JBox2D yet, but
				// when they are, commenting out the following section should work.
			/*
			 * jointValue.put("type", "gear");
			 *
			 * GearJoint gearJoint = (GearJoint)joint; int jointIndex1 =
			 * lookupJointIndex( gearJoint.getJoint1() ); int jointIndex2 =
			 * lookupJointIndex( gearJoint.getJoint2() );
			 * jointValue.put("joint1", jointIndex1); jointValue.put("joint2",
			 * jointIndex2); jointValue.put("ratio", gearJoint.getRatio());
			 */
			}
			break;
			case WheelJoint: {
				// Wheel joints are apparently not implemented in JBox2D yet, but
				// when they are, commenting out the following section... might
				// work.
			/*
			 * jointValue.put("type", "wheel");
			 *
			 * WheelJoint wheelJoint = (WheelJoint)joint;
			 * wheelJoint.getAnchorA(tmpAnchor); vecToJson("anchorA",
			 * bodyA.getLocalPoint(tmpAnchor), jointValue);
			 * wheelJoint.getAnchorB(tmpAnchor); vecToJson("anchorB",
			 * bodyB.getLocalPoint(tmpAnchor), jointValue);
			 * vecToJson("localAxisA", wheelJoint.getLocalAxisA(), jointValue);
			 * jointValue.put("enableMotor", wheelJoint.isMotorEnabled());
			 * floatToJson("motorSpeed", wheelJoint.getMotorSpeed(),
			 * jointValue); floatToJson("maxMotorTorque",
			 * wheelJoint.getMaxMotorTorque(), jointValue);
			 * floatToJson("springFrequency", wheelJoint.getSpringFrequencyHz(),
			 * jointValue); floatToJson("springDampingRatio",
			 * wheelJoint.getSpringDampingRatio(), jointValue);
			 */
			}
			break;
			case WeldJoint: {
				jointValue.put("type", "weld");

				WeldJoint weldJoint = (WeldJoint) joint;
				tmpAnchor = weldJoint.getAnchorA();
				vecToJson("anchorA", bodyA.getLocalPoint(tmpAnchor), jointValue);
				tmpAnchor = weldJoint.getAnchorB();
				vecToJson("anchorB", bodyB.getLocalPoint(tmpAnchor), jointValue);
/*
				floatToJson("refAngle", weldJoint.getReferenceAngle(), jointValue);
*/
			}
			break;
			case FrictionJoint: {
				jointValue.put("type", "friction");

				FrictionJoint frictionJoint = (FrictionJoint) joint;
				tmpAnchor = frictionJoint.getAnchorA();
				vecToJson("anchorA", bodyA.getLocalPoint(tmpAnchor), jointValue);
				tmpAnchor = frictionJoint.getAnchorB();
				vecToJson("anchorB", bodyB.getLocalPoint(tmpAnchor), jointValue);
				floatToJson("maxForce", frictionJoint.getMaxForce(), jointValue);
				floatToJson("maxTorque", frictionJoint.getMaxTorque(), jointValue);
			}
			break;
			case RopeJoint: {
				// Rope joints are apparently not implemented in JBox2D yet, but
				// when they are, commenting out the following section should work.

				jointValue.put("type", "rope");

				RopeJoint ropeJoint = (RopeJoint) joint;
				tmpAnchor = ropeJoint.getAnchorA();
				vecToJson("anchorA",
						bodyA.getLocalPoint(tmpAnchor), jointValue);
				tmpAnchor = ropeJoint.getAnchorB();
				vecToJson("anchorB",
						bodyB.getLocalPoint(tmpAnchor), jointValue);
				floatToJson("maxLength", ropeJoint.getMaxLength(), jointValue);

			}
			break;
			case Unknown:
			default:
				System.out.println("Unknown joint type : " + joint.getType());
		}

		JSONArray customPropertyValue = writeCustomPropertiesToJson(joint);
		if (customPropertyValue.length() > 0)
			jointValue.put("customProperties", customPropertyValue);

		return jointValue;
	}

	JSONObject b2j(Jb2dJsonImage image) throws JSONException {
		JSONObject imageValue = new JSONObject();

		if (null != image.body)
			imageValue.put("body", lookupBodyIndex(image.body));
		else
			imageValue.put("body", -1);

		if (null != image.name)
			imageValue.put("name", image.name);
		if (null != image.file)
			imageValue.put("file", image.file);

		vecToJson("center", image.center, imageValue);
		floatToJson("angle", image.angle, imageValue);
		floatToJson("scale", image.scale, imageValue);
		floatToJson("aspectScale", image.aspectScale, imageValue);
		if (image.flip)
			imageValue.put("flip", true);
		floatToJson("opacity", image.opacity, imageValue);
		imageValue.put("filter", image.filter);
		floatToJson("renderOrder", image.renderOrder, imageValue);

		boolean defaultColorTint = true;
		for (int i = 0; i < 4; i++) {
			if (image.colorTint[i] != 255) {
				defaultColorTint = false;
				break;
			}
		}

		if (!defaultColorTint) {
			JSONArray array = imageValue.getJSONArray("colorTint");
			for (int i = 0; i < 4; i++)
				array.put(i, image.colorTint[i]);
		}

		// image->updateCorners();
		for (int i = 0; i < 4; i++)
			vecToJson("corners", image.corners[i], imageValue, i);

		// image->updateUVs();
		for (int i = 0; i < 2 * image.numPoints; i++) {
			vecToJson("glVertexPointer", image.points[i], imageValue, i);
			vecToJson("glTexCoordPointer", image.uvCoords[i], imageValue, i);
		}
		for (int i = 0; i < image.numIndices; i++)
			vecToJson("glDrawElements", image.indices[i], imageValue, i);

		JSONArray customPropertyValue = writeCustomPropertiesToJson(image);
		if (customPropertyValue.length() > 0)
			imageValue.put("customProperties", customPropertyValue);

		return imageValue;
	}

	Body lookupBodyFromIndex(int index) {
		if (m_indexToBodyMap.containsKey(index))
			return m_indexToBodyMap.get(index);
		else
			return null;
	}

	protected int lookupBodyIndex(Body body) {
		Integer val = m_bodyToIndexMap.get(body);
		if (null != val)
			return val.intValue();
		else
			return -1;
	}

	protected int lookupJointIndex(Joint joint) {
		Integer val = m_jointToIndexMap.get(joint);
		if (null != val)
			return val.intValue();
		else
			return -1;
	}

	public String getBodyName(Body body) {
		return m_bodyToNameMap.get(body);
	}

	public String getFixtureName(Fixture fixture) {
		return m_fixtureToNameMap.get(fixture);
	}

	public String getJointName(Joint joint) {
		return m_jointToNameMap.get(joint);
	}

	public String getImageName(Jb2dJsonImage image) {
		return m_imageToNameMap.get(image);
	}

	public String floatToHex(float f) {
		int bits = Float.floatToIntBits(f);
		return Integer.toHexString(bits);
	}

	public void floatToJson(String name, float f, JSONObject value) throws JSONException {
		// cut down on file space for common values
		if (f == 0)
			value.put(name, 0);
		else if (f == 1)
			value.put(name, 1);
		else {
			if (m_useHumanReadableFloats)
				value.put(name, f);
			else
				value.put(name, floatToHex(f));
		}
	}

	public void vecToJson(String name, int v, JSONObject value, int index) throws JSONException {
		if (index > -1) {
			JSONArray array = value.getJSONArray(name);
			array.put(index, v);
		} else
			value.put(name, v);
	}

	public void vecToJson(String name, float v, JSONObject value, int index) throws JSONException {
		if (index > -1) {
			if (m_useHumanReadableFloats) {
				JSONArray array = value.getJSONArray(name);
				array.put(index, v);
			} else {
				JSONArray array = value.getJSONArray(name);
				if (v == 0)
					array.put(index, 0);
				else if (v == 1)
					array.put(index, 1);
				else
					array.put(index, floatToHex(v));
			}
		} else
			floatToJson(name, v, value);
	}

	public void vecToJson(String name, Vector2 vec, JSONObject value) throws JSONException {
		vecToJson(name, vec, value, -1);
	}

	public void vecToJson(String name, Vector2 vec, JSONObject value, int index) throws JSONException {
		if (index > -1) {
			if (m_useHumanReadableFloats) {
				boolean alreadyHadArray = value.has(name);
				JSONArray arrayX = alreadyHadArray ? value.getJSONObject(name).getJSONArray("x") : new JSONArray();
				JSONArray arrayY = alreadyHadArray ? value.getJSONObject(name).getJSONArray("y") : new JSONArray();
				arrayX.put(index, vec.x);
				arrayY.put(index, vec.y);
				if (!alreadyHadArray) {
					JSONObject subValue = new JSONObject();
					subValue.put("x", arrayX);
					subValue.put("y", arrayY);
					value.put(name, subValue);
				}
			} else {
				boolean alreadyHadArray = value.has(name);
				JSONArray arrayX = alreadyHadArray ? value.getJSONObject(name).getJSONArray("x") : new JSONArray();
				JSONArray arrayY = alreadyHadArray ? value.getJSONObject(name).getJSONArray("y") : new JSONArray();
				if (vec.x == 0)
					arrayX.put(index, 0);
				else if (vec.x == 1)
					arrayX.put(index, 1);
				else
					arrayX.put(index, floatToHex(vec.x));
				if (vec.y == 0)
					arrayY.put(index, 0);
				else if (vec.y == 1)
					arrayY.put(index, 1);
				else
					arrayY.put(index, floatToHex(vec.y));
				if (!alreadyHadArray) {
					JSONObject subValue = new JSONObject();
					subValue.put("x", arrayX);
					subValue.put("y", arrayY);
					value.put(name, subValue);
				}
			}
		} else {
			if (vec.x == 0 && vec.y == 0)
				value.put(name, 0);// cut down on file space for common values
			else {
				JSONObject vecValue = new JSONObject();
				floatToJson("x", vec.x, vecValue);
				floatToJson("y", vec.y, vecValue);
				value.put(name, vecValue);
			}
		}
	}

	public void clear() {
		m_indexToBodyMap.clear();
		m_bodyToIndexMap.clear();
		m_jointToIndexMap.clear();
		m_bodies.clear();
		m_joints.clear();
		m_images.clear();

		m_bodyToNameMap.clear();
		m_fixtureToNameMap.clear();
		m_jointToNameMap.clear();
		m_imageToNameMap.clear();
	}

	// Pass null for existingWorld to create a new world
	public World readFromJSONObject(JSONObject worldValue, World existingWorld) throws JSONException {
		clear();

		return j2b2World(worldValue, existingWorld);
	}

	// Pass null for existingWorld to create a new world
	public World readFromString(String str, StringBuilder errorMsg, World existingWorld) {
		try {
			JSONObject worldValue = new JSONObject(str);
			return j2b2World(worldValue, existingWorld);
		} catch (JSONException e) {
			errorMsg.append("Failed to parse JSON");
			e.printStackTrace();
			return null;
		}
	}

	// Pass null for existingWorld to create a new world
	public World readFromFile(FileHandle h, StringBuilder errorMsg, World existingWorld) {
		if (null == h)
			return null;

		String str;
		str = h.readString("UTF-8"); //todo figure out charsets
//		str = h.readString();
		try {
			JSONObject worldValue = new JSONObject(str);
			return world = j2b2World(worldValue, existingWorld);
		} catch (JSONException e) {
			errorMsg.append("\nFailed to parse JSON: " + h);
			e.printStackTrace();
			return null;
		}
	}

	public World j2b2World(JSONObject worldValue, World existingWorld) throws JSONException {
		World world = existingWorld;
		if (world == null)
			world = new World(jsonToVec("gravity", worldValue), worldValue.getBoolean("allowSleep"));

		world.setAutoClearForces(worldValue.getBoolean("autoClearForces"));
		world.setWarmStarting(worldValue.getBoolean("warmStarting"));
		world.setContinuousPhysics(worldValue.getBoolean("continuousPhysics"));
		// world.setSubStepping( worldValue.getBoolean("subStepping") );

		readCustomPropertiesFromJson(world, worldValue);

		int i = 0;
		JSONArray bodyValues = worldValue.optJSONArray("body");
		if (null != bodyValues) {
			int numBodyValues = bodyValues.length();
			for (i = 0; i < numBodyValues; i++) {
				JSONObject bodyValue = bodyValues.getJSONObject(i);
				Body body = j2b2Body(world, bodyValue);
				readCustomPropertiesFromJson(body, bodyValue);
				m_bodies.add(body);
				m_indexToBodyMap.put(i, body);
			}
		}

		// need two passes for joints because gear joints reference other joints
		JSONArray jointValues = worldValue.optJSONArray("joint");
		if (null != jointValues) {
			int numJointValues = jointValues.length();
			for (i = 0; i < numJointValues; i++) {
				JSONObject jointValue = jointValues.getJSONObject(i);
				if (!jointValue.optString("type", "").equals("gear")) {
					Joint joint = j2b2Joint(world, jointValue);
					readCustomPropertiesFromJson(joint, jointValue);
					m_joints.add(joint);
				}
			}
			for (i = 0; i < numJointValues; i++) {
				JSONObject jointValue = jointValues.getJSONObject(i);
				if (jointValue.optString("type", "").equals("gear")) {
					Joint joint = j2b2Joint(world, jointValue);
					readCustomPropertiesFromJson(joint, jointValue);
					m_joints.add(joint);
				}
			}
		}

		i = 0;
		JSONArray imageValues = worldValue.optJSONArray("image");
		if (null != imageValues) {
			int numImageValues = imageValues.length();
			for (i = 0; i < numImageValues; i++) {
				JSONObject imageValue = imageValues.getJSONObject(i);
				Jb2dJsonImage image = j2b2dJsonImage(imageValue);
				readCustomPropertiesFromJson(image, imageValue);
				m_images.add(image);
			}
		}

		return world;
	}

	public Body j2b2Body(World world, JSONObject bodyValue) throws JSONException {
		BodyDef bodyDef = new BodyDef();
		switch (bodyValue.getInt("type")) {
			case 0:
				bodyDef.type = BodyDef.BodyType.StaticBody;
				break;
			case 1:
				bodyDef.type = BodyDef.BodyType.KinematicBody;
				break;
			case 2:
				bodyDef.type = BodyDef.BodyType.DynamicBody;
				break;
		}
		bodyDef.position.set(jsonToVec("position", bodyValue));
		bodyDef.angle = jsonToFloat("angle", bodyValue);
		bodyDef.linearVelocity.set(jsonToVec("linearVelocity", bodyValue));
		bodyDef.angularVelocity = jsonToFloat("angularVelocity", bodyValue);
		bodyDef.linearDamping = jsonToFloat("linearDamping", bodyValue, -1, 0);
		bodyDef.angularDamping = jsonToFloat("angularDamping", bodyValue, -1, 0);
		bodyDef.gravityScale = jsonToFloat("gravityScale", bodyValue, -1, 1);

		bodyDef.allowSleep = bodyValue.optBoolean("allowSleep", true);
		bodyDef.awake = bodyValue.optBoolean("awake", false);
		bodyDef.fixedRotation = bodyValue.optBoolean("fixedRotation");
		bodyDef.bullet = bodyValue.optBoolean("bullet", false);
		bodyDef.active = bodyValue.optBoolean("active", true);

		Body body = world.createBody(bodyDef);

		String bodyName = bodyValue.optString("name", "");
		if ("" != bodyName)
			setBodyName(body, bodyName);

		int i = 0;
		JSONArray fixtureValues = bodyValue.optJSONArray("fixture");
		if (null != fixtureValues) {
			int numFixtureValues = fixtureValues.length();
			for (i = 0; i < numFixtureValues; i++) {
				JSONObject fixtureValue = fixtureValues.getJSONObject(i);
				Fixture fixture = j2b2Fixture(body, fixtureValue);
				readCustomPropertiesFromJson(fixture, fixtureValue);
			}
		}

		// may be necessary if user has overridden mass characteristics
		MassData massData = new MassData();
		massData.mass = jsonToFloat("massData-mass", bodyValue);
		massData.center.set(jsonToVec("massData-center", bodyValue));
		massData.I = jsonToFloat("massData-I", bodyValue);
		body.setMassData(massData);

		return body;
	}

	Fixture j2b2Fixture(Body body, JSONObject fixtureValue) throws JSONException {

		if (null == fixtureValue)
			return null;

		FixtureDef fixtureDef = new FixtureDef();
		fixtureDef.restitution = jsonToFloat("restitution", fixtureValue);
		fixtureDef.friction = jsonToFloat("friction", fixtureValue);
		fixtureDef.density = jsonToFloat("density", fixtureValue);
		fixtureDef.isSensor = fixtureValue.optBoolean("sensor", false);

		fixtureDef.filter.categoryBits = (short) fixtureValue.optInt("filter-categoryBits", 0x0001);
		fixtureDef.filter.maskBits = (short) fixtureValue.optInt("filter-maskBits", 0xffff);
		fixtureDef.filter.groupIndex = (short) fixtureValue.optInt("filter-groupIndex", 0);

		Fixture fixture = null;
		if (null != fixtureValue.optJSONObject("circle")) {
			JSONObject circleValue = fixtureValue.getJSONObject("circle");
			CircleShape circleShape = new CircleShape();
			circleShape.setRadius(jsonToFloat("radius", circleValue));
			circleShape.setPosition(jsonToVec("center", circleValue));
			fixtureDef.shape = circleShape;
			fixture = body.createFixture(fixtureDef);
		} else if (null != fixtureValue.optJSONObject("edge")) {
			JSONObject edgeValue = fixtureValue.getJSONObject("edge");
			EdgeShape edgeShape = new EdgeShape();
			edgeShape.set(jsonToVec("vertex1", edgeValue), jsonToVec("vertex2", edgeValue));
			edgeShape.setHasVertex0(edgeValue.optBoolean("hasVertex0", false));
			edgeShape.setHasVertex3(edgeValue.optBoolean("hasVertex3", false));
			if (edgeShape.hasVertex0())
				edgeShape.setVertex0(jsonToVec("vertex0", edgeValue));
			if (edgeShape.hasVertex3())
				edgeShape.setVertex3(jsonToVec("vertex3", edgeValue));
			fixtureDef.shape = edgeShape;
			fixture = body.createFixture(fixtureDef);
		} else if (null != fixtureValue.optJSONObject("loop")) {// support old
			// format (r197)
			JSONObject chainValue = fixtureValue.getJSONObject("loop");
			ChainShape chainShape = new ChainShape();
			int numVertices = chainValue.getJSONArray("x").length();
			Vector2 vertices[] = new Vector2[numVertices];
			for (int i = 0; i < numVertices; i++)
				vertices[i].set(jsonToVec("vertices", chainValue, i));
			chainShape.createLoop(vertices);
			fixtureDef.shape = chainShape;
			fixture = body.createFixture(fixtureDef);
		} else if (null != fixtureValue.optJSONObject("chain")) {
			JSONObject chainValue = fixtureValue.getJSONObject("chain");
			ChainShape chainShape = new ChainShape();
			int numVertices = chainValue.getJSONObject("vertices").getJSONArray("x").length();
			Vector2 vertices[] = new Vector2[numVertices];
			for (int i = 0; i < numVertices; i++)
				vertices[i] = jsonToVec("vertices", chainValue, i);
			chainShape.createChain(vertices);
/*
			chainShape.m_hasPrevVertex = chainValue.optBoolean("hasPrevVertex", false);
			chainShape.m_hasNextVertex = chainValue.optBoolean("hasNextVertex", false);
			if (chainShape.m_hasPrevVertex)
				chainShape.m_prevVertex.set(jsonToVec("prevVertex", chainValue));
			if (chainShape.m_hasNextVertex)
				chainShape.m_nextVertex.set(jsonToVec("nextVertex", chainValue));
*/
			fixtureDef.shape = chainShape;
			fixture = body.createFixture(fixtureDef);
		} else if (null != fixtureValue.optJSONObject("polygon")) {
			JSONObject polygonValue = fixtureValue.getJSONObject("polygon");
			int numVertices = polygonValue.getJSONObject("vertices").getJSONArray("x").length();
			Vector2 vertices[] = new Vector2[numVertices];
			if (numVertices > maxPolygonVertices) {
				System.out.println("Ignoring polygon fixture with too many vertices.");
			} else if (numVertices < 2) {
				System.out.println("Ignoring polygon fixture less than two vertices.");
			} else if (numVertices == 2) {
				System.out.println("Creating edge shape instead of polygon with two vertices.");
				EdgeShape edgeShape = new EdgeShape();
				edgeShape.set(jsonToVec("vertices", polygonValue, 0), jsonToVec("vertices", polygonValue, 1));
				fixtureDef.shape = edgeShape;
				fixture = body.createFixture(fixtureDef);
			} else {
				PolygonShape polygonShape = new PolygonShape();
				for (int i = 0; i < numVertices; i++)
					vertices[i] = jsonToVec("vertices", polygonValue, i);
				polygonShape.set(vertices);
				fixtureDef.shape = polygonShape;
				fixture = body.createFixture(fixtureDef);
			}
		}

		String fixtureName = fixtureValue.optString("name", "");
		if (fixtureName != "") {
			setFixtureName(fixture, fixtureName);
		}

		return fixture;
	}

	Joint j2b2Joint(World world, JSONObject jointValue) throws JSONException {
		Joint joint = null;

		int bodyIndexA = jointValue.getInt("bodyA");
		int bodyIndexB = jointValue.getInt("bodyB");
		if (bodyIndexA >= m_bodies.size || bodyIndexB >= m_bodies.size)
			return null;

		// keep these in scope after the if/else below
		RevoluteJointDef revoluteDef;
		PrismaticJointDef prismaticDef;
		DistanceJointDef distanceDef;
		PulleyJointDef pulleyDef;
		MouseJointDef mouseDef;
		GearJointDef gearDef;
		// WheelJointDef wheelDef;
		WeldJointDef weldDef;
		FrictionJointDef frictionDef;
		RopeJointDef ropeDef;

		// will be used to select one of the above to work with
		JointDef jointDef = null;

		Vector2 mouseJointTarget = new Vector2(0, 0);
		String type = jointValue.optString("type", "");
		if (type.equals("revolute")) {
			jointDef = revoluteDef = new RevoluteJointDef();
			revoluteDef.localAnchorA.set(jsonToVec("anchorA", jointValue));
			revoluteDef.localAnchorB.set(jsonToVec("anchorB", jointValue));
			revoluteDef.referenceAngle = jsonToFloat("refAngle", jointValue);
			revoluteDef.enableLimit = jointValue.optBoolean("enableLimit", false);
			revoluteDef.lowerAngle = jsonToFloat("lowerLimit", jointValue);
			revoluteDef.upperAngle = jsonToFloat("upperLimit", jointValue);
			revoluteDef.enableMotor = jointValue.optBoolean("enableMotor", false);
			revoluteDef.motorSpeed = jsonToFloat("motorSpeed", jointValue);
			revoluteDef.maxMotorTorque = jsonToFloat("maxMotorTorque", jointValue);
		} else if (type.equals("prismatic")) {
			jointDef = prismaticDef = new PrismaticJointDef();
			prismaticDef.localAnchorA.set(jsonToVec("anchorA", jointValue));
			prismaticDef.localAnchorB.set(jsonToVec("anchorB", jointValue));
			if (jointValue.has("localAxisA"))
				prismaticDef.localAxisA.set(jsonToVec("localAxisA", jointValue));
			else
				prismaticDef.localAxisA.set(jsonToVec("localAxis1", jointValue));
			prismaticDef.referenceAngle = jsonToFloat("refAngle", jointValue);
			prismaticDef.enableLimit = jointValue.optBoolean("enableLimit");
			prismaticDef.lowerTranslation = jsonToFloat("lowerLimit", jointValue);
			prismaticDef.upperTranslation = jsonToFloat("upperLimit", jointValue);
			prismaticDef.enableMotor = jointValue.optBoolean("enableMotor");
			prismaticDef.motorSpeed = jsonToFloat("motorSpeed", jointValue);
			prismaticDef.maxMotorForce = jsonToFloat("maxMotorForce", jointValue);
		} else if (type.equals("distance")) {
			jointDef = distanceDef = new DistanceJointDef();
			distanceDef.localAnchorA.set(jsonToVec("anchorA", jointValue));
			distanceDef.localAnchorB.set(jsonToVec("anchorB", jointValue));
			distanceDef.length = jsonToFloat("length", jointValue);
			distanceDef.frequencyHz = jsonToFloat("frequency", jointValue);
			distanceDef.dampingRatio = jsonToFloat("dampingRatio", jointValue);
		} else if (type.equals("pulley")) {
			jointDef = pulleyDef = new PulleyJointDef();
			pulleyDef.groundAnchorA.set(jsonToVec("groundAnchorA", jointValue));
			pulleyDef.groundAnchorB.set(jsonToVec("groundAnchorB", jointValue));
			pulleyDef.localAnchorA.set(jsonToVec("anchorA", jointValue));
			pulleyDef.localAnchorB.set(jsonToVec("anchorB", jointValue));
			pulleyDef.lengthA = jsonToFloat("lengthA", jointValue);
			pulleyDef.lengthB = jsonToFloat("lengthB", jointValue);
			pulleyDef.ratio = jsonToFloat("ratio", jointValue);
		} else if (type.equals("mouse")) {
			jointDef = mouseDef = new MouseJointDef();
			mouseJointTarget = jsonToVec("target", jointValue);
			mouseDef.target.set(jsonToVec("anchorB", jointValue));// alter after creating joint
			mouseDef.maxForce = jsonToFloat("maxForce", jointValue);
			mouseDef.frequencyHz = jsonToFloat("frequency", jointValue);
			mouseDef.dampingRatio = jsonToFloat("dampingRatio", jointValue);
		}
		// Gear joints are apparently not implemented in JBox2D yet, but
		// when they are, commenting out the following section should work.
		/*
		 * else if ( type.equals("gear") ) { jointDef = gearDef = new
		 * GearJointDef(); int jointIndex1 = jointValue.getInt("joint1"); int
		 * jointIndex2 = jointValue.getInt("joint2"); gearDef.joint1 =
		 * m_joints.get(jointIndex1); gearDef.joint2 =
		 * m_joints.get(jointIndex2); gearDef.ratio = jsonToFloat("ratio",
		 * jointValue); }
		 */
		// Wheel joints are apparently not implemented in JBox2D yet, but
		// when they are, commenting out the following section should work.
		/*
		 * else if ( type.equals("wheel") ) { jointDef = wheelDef = new
		 * WheelJointDef(); wheelDef.localAnchorA.set( jsonToVec("anchorA",
		 * jointValue) ); wheelDef.localAnchorB.set( jsonToVec("anchorB",
		 * jointValue) ); wheelDef.localAxisA.set( jsonToVec("localAxisA",
		 * jointValue) ); wheelDef.enableMotor =
		 * jointValue.optBoolean("enableMotor",false); wheelDef.motorSpeed =
		 * jsonToFloat("motorSpeed", jointValue); wheelDef.maxMotorTorque =
		 * jsonToFloat("maxMotorTorque", jointValue); wheelDef.frequencyHz =
		 * jsonToFloat("springFrequency", jointValue); wheelDef.dampingRatio =
		 * jsonToFloat("springDampingRatio", jointValue); }
		 */
		// For now, we will make do with a revolute joint.
		else if (type.equals("wheel")) {
			jointDef = revoluteDef = new RevoluteJointDef();
			revoluteDef.localAnchorA.set(jsonToVec("anchorA", jointValue));
			revoluteDef.localAnchorB.set(jsonToVec("anchorB", jointValue));
			revoluteDef.enableMotor = jointValue.optBoolean("enableMotor", false);
			revoluteDef.motorSpeed = jsonToFloat("motorSpeed", jointValue);
			revoluteDef.maxMotorTorque = jsonToFloat("maxMotorTorque", jointValue);
		} else if (type.equals("weld")) {
			jointDef = weldDef = new WeldJointDef();
			weldDef.localAnchorA.set(jsonToVec("anchorA", jointValue));
			weldDef.localAnchorB.set(jsonToVec("anchorB", jointValue));
			weldDef.referenceAngle = 0;
		} else if (type.equals("friction")) {
			jointDef = frictionDef = new FrictionJointDef();
			frictionDef.localAnchorA.set(jsonToVec("anchorA", jointValue));
			frictionDef.localAnchorB.set(jsonToVec("anchorB", jointValue));
			frictionDef.maxForce = jsonToFloat("maxForce", jointValue);
			frictionDef.maxTorque = jsonToFloat("maxTorque", jointValue);
		} else if (type.equals("rope")) {
			jointDef = ropeDef = new RopeJointDef();
			ropeDef.localAnchorA.set(jsonToVec("anchorA", jointValue));
			ropeDef.localAnchorB.set(jsonToVec("anchorB", jointValue));
			ropeDef.maxLength = jsonToFloat("maxLength", jointValue);
		}


		if (null != jointDef) {
			// set features common to all joints
			jointDef.bodyA = m_bodies.get(bodyIndexA);
			jointDef.bodyB = m_bodies.get(bodyIndexB);
			jointDef.collideConnected = jointValue.optBoolean("collideConnected", false);

			joint = world.createJoint(jointDef);

			if (type.equals("mouse"))
				((MouseJoint) joint).setTarget(mouseJointTarget);

			String jointName = jointValue.optString("name", "");
			if (!jointName.equals("")) {
				setJointName(joint, jointName);
			}
		}

		return joint;
	}

	Jb2dJsonImage j2b2dJsonImage(JSONObject imageValue) throws JSONException {
		Jb2dJsonImage img = new Jb2dJsonImage();

		int bodyIndex = imageValue.optInt("body", -1);
		if (-1 != bodyIndex)
			img.body = lookupBodyFromIndex(bodyIndex);

		String imageName = imageValue.optString("name", "");
		if (!imageName.equals("")) {
			img.name = imageName;
			setImageName(img, imageName);
		}

		String fileName = imageValue.optString("file", "");
		if (!fileName.equals(""))
			img.file = fileName;

		img.center = jsonToVec("center", imageValue);
		img.angle = jsonToFloat("angle", imageValue);
		img.scale = jsonToFloat("scale", imageValue);
		img.aspectScale = jsonToFloat("aspectScale", imageValue);
		img.opacity = jsonToFloat("opacity", imageValue);
		img.renderOrder = jsonToFloat("renderOrder", imageValue);

		JSONArray colorTintArray = imageValue.optJSONArray("colorTint");
		if (null != colorTintArray) {
			for (int i = 0; i < 4; i++) {
				img.colorTint[i] = colorTintArray.getInt(i);
			}
		}

		img.flip = imageValue.optBoolean("flip", false);

		img.filter = imageValue.optInt("filter", 1);

		img.corners = new Vector2[4];
		for (int i = 0; i < 4; i++)
			img.corners[i] = jsonToVec("corners", imageValue, i);

		JSONArray vertexPointerArray = imageValue.optJSONArray("glVertexPointer");
		JSONArray texCoordArray = imageValue.optJSONArray("glVertexPointer");
		if (null != vertexPointerArray && null != texCoordArray && vertexPointerArray.length() == texCoordArray.length()) {
			int numFloats = vertexPointerArray.length();
			img.numPoints = numFloats / 2;
			img.points = new float[numFloats];
			img.uvCoords = new float[numFloats];
			for (int i = 0; i < numFloats; i++) {
				img.points[i] = jsonToFloat("glVertexPointer", imageValue, i);
				img.uvCoords[i] = jsonToFloat("glTexCoordPointer", imageValue, i);
			}
		}

		JSONArray drawElementsArray = imageValue.optJSONArray("glDrawElements");
		if (null != drawElementsArray) {
			img.numIndices = drawElementsArray.length();
			img.indices = new short[img.numIndices];
			for (int i = 0; i < img.numIndices; i++)
				img.indices[i] = (short) drawElementsArray.getInt(i);
		}

		return img;
	}

	float jsonToFloat(String name, JSONObject value) {
		return jsonToFloat(name, value, -1, 0);
	}

	float jsonToFloat(String name, JSONObject value, int index) {
		return jsonToFloat(name, value, index, 0);
	}

	float jsonToFloat(String name, JSONObject value, int index, float defaultValue) {
		if (!value.has(name))
			return defaultValue;

		if (index > -1) {
			JSONArray array = null;
			try {
				array = value.getJSONArray(name);
			} catch (JSONException e) {
			}
			if (null == array)
				return defaultValue;
			Object obj = array.opt(index);
			if (null == obj)
				return defaultValue;
				// else if ( value[name].isString() )
				// return hexToFloat( value[name].asString() );
			else
				return ((Number) obj).floatValue();
		} else {
			Object obj = value.opt(name);
			if (null == obj)
				return defaultValue;
				// else if ( value[name].isString() )
				// return hexToFloat( value[name].asString() );
			else
				return ((Number) obj).floatValue();
		}
	}

	Vector2 jsonToVec(String name, JSONObject value) throws JSONException {
		return jsonToVec(name, value, -1, new Vector2(0, 0));
	}

	Vector2 jsonToVec(String name, JSONObject value, int index) throws JSONException {
		return jsonToVec(name, value, index, new Vector2(0, 0));
	}

	Vector2 jsonToVec(String name, JSONObject value, int index, Vector2 defaultValue) throws JSONException {
		Vector2 vec = defaultValue;

		if (!value.has(name))
			return defaultValue;

		if (index > -1) {
			JSONObject vecValue = value.getJSONObject(name);
			JSONArray arrayX = vecValue.getJSONArray("x");
			JSONArray arrayY = vecValue.getJSONArray("y");
			// if ( arrayX[index].isString() )
			// vec.x = hexToFloat(value[name]["x"][index].asString());
			// else
			vec.x = (float) arrayX.getDouble(index);

			// if ( arrayX[index].isString() )
			// vec.y = hexToFloat(value[name]["y"][index].asString());
			// else
			vec.y = (float) arrayY.getDouble(index);
		} else {
			JSONObject vecValue = value.optJSONObject(name);
			if (null == vecValue)
				return defaultValue;
			else if (!vecValue.has("x")) // should be zero vector
				vec.set(0, 0);
			else {
				vec.x = jsonToFloat("x", vecValue);
				vec.y = jsonToFloat("y", vecValue);
			}
		}

		return vec;
	}

	public Body[] getBodiesByName(String name) {
		Set<Body> keys = new HashSet<Body>();
		for (ObjectMap.Entry<Body, String> entry : m_bodyToNameMap.entries()) {
			if (name.equals(entry.value)) {
				keys.add(entry.key);
			}
		}
		return keys.toArray(new Body[0]);
	}

	public Fixture[] getFixturesByName(String name) {
		Set<Fixture> keys = new HashSet<Fixture>();
		for (ObjectMap.Entry<Fixture, String> entry : m_fixtureToNameMap.entries()) {
			if (name.equals(entry.value)) {
				keys.add(entry.key);
			}
		}
		return keys.toArray(new Fixture[0]);
	}

	public Joint[] getJointsByName(String name) {
		Set<Joint> keys = new HashSet<Joint>();
		for (ObjectMap.Entry<Joint, String> entry : m_jointToNameMap.entries()) {
			if (name.equals(entry.value)) {
				keys.add(entry.key);
			}
		}
		return keys.toArray(new Joint[0]);
	}

	public Jb2dJsonImage[] getImagesByName(String name) {
		Set<Jb2dJsonImage> keys = new HashSet<Jb2dJsonImage>();
		for (ObjectMap.Entry<Jb2dJsonImage, String> entry : m_imageToNameMap.entries()) {
			if (name.equals(entry.value)) {
				keys.add(entry.key);
			}
		}
		return keys.toArray(new Jb2dJsonImage[0]);
	}

	public Array<Jb2dJsonImage> getAllImages() {
		return m_images;
	}

	/**
	 * Get the images which are tied to the specified body. Pass null for unattached images.
	 * @param body box2d body image refers to
	 * @return array containing naturally ordered images
	 */
	public Jb2dJsonImage[] getImagesForBody(Body body) {
		Set<Jb2dJsonImage> keys = new HashSet<Jb2dJsonImage>();
		for (ObjectMap.Entry<Jb2dJsonImage, String> entry : m_imageToNameMap.entries()) {
			if (body==null&&entry.key.body == null||body!=null&&body.equals(entry.key.body))
				keys.add(entry.key);
		}
		return keys.toArray(new Jb2dJsonImage[0]);
	}

	/**
	 * Get the images which are tied to the specified body. Pass null for unattached images.
	 * @param body box2d body image refers to
	 * @param arr OrderedMap in which to add the images which refer to the body
	 * @return the size of the MrderedMap
	 */
	public int getImagesForBody(Body body, Array<Jb2dJsonImage> arr) {
		for (ObjectMap.Entry<Jb2dJsonImage, String> entry : m_imageToNameMap.entries()) {
			if (body==null&&entry.key.body == null||body!=null&&body.equals(entry.key.body))
				arr.add(entry.key);
		}
		return arr.size;
	}

	public Body getBodyByName(String name) {
		for (ObjectMap.Entry<Body, String> entry : m_bodyToNameMap.entries()) {
			if (name.equals(entry.value)) {
				return entry.key;
			}
		}
		return null;
	}

	public Fixture getFixtureByName(String name) {
		for (ObjectMap.Entry<Fixture, String> entry : m_fixtureToNameMap.entries()) {
			if (name.equals(entry.value)) {
				return entry.key;
			}
		}
		return null;
	}

	public Joint getJointByName(String name) {
		for (ObjectMap.Entry<Joint, String> entry : m_jointToNameMap.entries()) {
			if (name.equals(entry.value)) {
				return entry.key;
			}
		}
		return null;
	}

	public Jb2dJsonImage getImageByName(String name) {
		for (ObjectMap.Entry<Jb2dJsonImage, String> entry : m_imageToNameMap.entries()) {
			if (name.equals(entry.value)) {
				return entry.key;
			}
		}
		return null;
	}

	// //// custom properties

	public Jb2dJsonCustomProperties getCustomPropertiesForItem(Object item, boolean createIfNotExisting) {

		if (m_customPropertiesMap.containsKey(item))
			return m_customPropertiesMap.get(item);

		if (!createIfNotExisting)
			return null;

		Jb2dJsonCustomProperties props = new Jb2dJsonCustomProperties();
		m_customPropertiesMap.put(item, props);

		return props;
	}

	// setCustomXXX

	protected void setCustomInt(Object item, String propertyName, int val) {
		getCustomPropertiesForItem(item, true).m_customPropertyMap_int.put(propertyName, val);
	}

	protected void setCustomFloat(Object item, String propertyName, float val) {
		getCustomPropertiesForItem(item, true).m_customPropertyMap_float.put(propertyName, new Double(val));
	}

	protected void setCustomString(Object item, String propertyName, String val) {
		getCustomPropertiesForItem(item, true).m_customPropertyMap_string.put(propertyName, val);
	}

	protected void setCustomVector(Object item, String propertyName, Vector2 val) {
		getCustomPropertiesForItem(item, true).m_customPropertyMap_vec2.put(propertyName, val);
	}

	protected void setCustomBool(Object item, String propertyName, boolean val) {
		getCustomPropertiesForItem(item, true).m_customPropertyMap_bool.put(propertyName, val);
	}


	public void setCustomInt(Body item, String propertyName, int val) {
		m_bodiesWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_int.put(propertyName, val);
	}

	public void setCustomFloat(Body item, String propertyName, float val) {
		m_bodiesWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_float.put(propertyName, new Double(val));
	}

	public void setCustomString(Body item, String propertyName, String val) {
		m_bodiesWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_string.put(propertyName, val);
	}

	public void setCustomVector(Body item, String propertyName, Vector2 val) {
		m_bodiesWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_vec2.put(propertyName, val);
	}

	public void setCustomBool(Body item, String propertyName, boolean val) {
		m_bodiesWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_bool.put(propertyName, val);
	}


	public void setCustomInt(Fixture item, String propertyName, int val) {
		m_fixturesWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_int.put(propertyName, val);
	}

	public void setCustomFloat(Fixture item, String propertyName, float val) {
		m_fixturesWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_float.put(propertyName, new Double(val));
	}

	public void setCustomString(Fixture item, String propertyName, String val) {
		m_fixturesWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_string.put(propertyName, val);
	}

	public void setCustomVector(Fixture item, String propertyName, Vector2 val) {
		m_fixturesWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_vec2.put(propertyName, val);
	}

	public void setCustomBool(Fixture item, String propertyName, boolean val) {
		m_fixturesWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_bool.put(propertyName, val);
	}


	public void setCustomInt(Joint item, String propertyName, int val) {
		m_jointsWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_int.put(propertyName, val);
	}

	public void setCustomFloat(Joint item, String propertyName, float val) {
		m_jointsWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_float.put(propertyName, new Double(val));
	}

	public void setCustomString(Joint item, String propertyName, String val) {
		m_jointsWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_string.put(propertyName, val);
	}

	public void setCustomVector(Joint item, String propertyName, Vector2 val) {
		m_jointsWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_vec2.put(propertyName, val);
	}

	public void setCustomBool(Joint item, String propertyName, boolean val) {
		m_jointsWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_bool.put(propertyName, val);
	}


	public void setCustomInt(Jb2dJsonImage item, String propertyName, int val) {
		m_imagesWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_int.put(propertyName, val);
	}

	public void setCustomFloat(Jb2dJsonImage item, String propertyName, float val) {
		m_imagesWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_float.put(propertyName, new Double(val));
	}

	public void setCustomString(Jb2dJsonImage item, String propertyName, String val) {
		m_imagesWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_string.put(propertyName, val);
	}

	public void setCustomVector(Jb2dJsonImage item, String propertyName, Vector2 val) {
		m_imagesWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_vec2.put(propertyName, val);
	}

	public void setCustomBool(Jb2dJsonImage item, String propertyName, boolean val) {
		m_imagesWithCustomProperties.add(item);
		getCustomPropertiesForItem(item, true).m_customPropertyMap_bool.put(propertyName, val);
	}


	// hasCustomXXX

	public boolean hasCustomInt(Object item, String propertyName) {
		return getCustomPropertiesForItem(item, false) != null &&
				getCustomPropertiesForItem(item, false).m_customPropertyMap_int.containsKey(propertyName);
	}

	public boolean hasCustomFloat(Object item, String propertyName) {
		return getCustomPropertiesForItem(item, false) != null &&
				getCustomPropertiesForItem(item, false).m_customPropertyMap_float.containsKey(propertyName);
	}

	public boolean hasCustomString(Object item, String propertyName) {
		return getCustomPropertiesForItem(item, false) != null &&
				getCustomPropertiesForItem(item, false).m_customPropertyMap_string.containsKey(propertyName);
	}

	public boolean hasCustomVector(Object item, String propertyName) {
		return getCustomPropertiesForItem(item, false) != null &&
				getCustomPropertiesForItem(item, false).m_customPropertyMap_vec2.containsKey(propertyName);
	}

	public boolean hasCustomBool(Object item, String propertyName) {
		return getCustomPropertiesForItem(item, false) != null &&
				getCustomPropertiesForItem(item, false).m_customPropertyMap_bool.containsKey(propertyName);
	}

	// getCustomXXX

	public int getCustomInt(Object item, String propertyName, int defaultVal) {
		Jb2dJsonCustomProperties props = getCustomPropertiesForItem(item, false);
		if (null == props)
			return defaultVal;
		if (props.m_customPropertyMap_int.containsKey(propertyName))
			return props.m_customPropertyMap_int.get(propertyName);
		return defaultVal;
	}

	public float getCustomFloat(Object item, String propertyName, float defaultVal) {
		Jb2dJsonCustomProperties props = getCustomPropertiesForItem(item, false);
		if (null == props)
			return defaultVal;
		if (props.m_customPropertyMap_float.containsKey(propertyName))
			return props.m_customPropertyMap_float.get(propertyName).floatValue();
		return defaultVal;
	}

	public String getCustomString(Object item, String propertyName, String defaultVal) {
		Jb2dJsonCustomProperties props = getCustomPropertiesForItem(item, false);
		if (null == props)
			return defaultVal;
		if (props.m_customPropertyMap_string.containsKey(propertyName))
			return props.m_customPropertyMap_string.get(propertyName);
		return defaultVal;
	}

	public Vector2 getCustomVector(Object item, String propertyName, Vector2 defaultVal) {
		Jb2dJsonCustomProperties props = getCustomPropertiesForItem(item, false);
		if (null == props)
			return defaultVal;
		if (props.m_customPropertyMap_vec2.containsKey(propertyName))
			return props.m_customPropertyMap_vec2.get(propertyName);
		return defaultVal;
	}

	public boolean getCustomBool(Object item, String propertyName, boolean defaultVal) {
		Jb2dJsonCustomProperties props = getCustomPropertiesForItem(item, false);
		if (null == props)
			return defaultVal;
		if (props.m_customPropertyMap_bool.containsKey(propertyName))
			return props.m_customPropertyMap_bool.get(propertyName);
		return defaultVal;
	}

	// get by custom property value (vector version, body)
	public int getBodiesByCustomInt(String propertyName, int valueToMatch, Vector<Body> items) {
		Iterator<Body> iterator = m_bodiesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Body item = iterator.next();
			if (hasCustomInt(item, propertyName) && getCustomInt(item, propertyName, 0) == valueToMatch)
				items.add(item);
		}
		return items.size();
	}

	public int getBodiesByCustomFloat(String propertyName, float valueToMatch, Vector<Body> items) {
		Iterator<Body> iterator = m_bodiesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Body item = iterator.next();
			if (hasCustomFloat(item, propertyName) && getCustomFloat(item, propertyName, 0) == valueToMatch)
				items.add(item);
		}
		return items.size();
	}

	public int getBodiesByCustomString(String propertyName, String valueToMatch, Vector<Body> items) {
		Iterator<Body> iterator = m_bodiesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Body item = iterator.next();
			if (hasCustomString(item, propertyName) && getCustomString(item, propertyName, new String()).equals(valueToMatch))
				items.add(item);
		}
		return items.size();
	}

	public int getBodiesByCustomVector(String propertyName, Vector2 valueToMatch, Vector<Body> items) {
		Iterator<Body> iterator = m_bodiesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Body item = iterator.next();
			if (hasCustomVector(item, propertyName) && getCustomVector(item, propertyName, new Vector2(0, 0)) == valueToMatch)
				items.add(item);
		}
		return items.size();
	}

	public int getBodiesByCustomBool(String propertyName, boolean valueToMatch, Vector<Body> items) {
		Iterator<Body> iterator = m_bodiesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Body item = iterator.next();
			if (hasCustomBool(item, propertyName) && getCustomBool(item, propertyName, false) == valueToMatch)
				items.add(item);
		}
		return items.size();
	}

	// get by custom property value (single version, body)
	Body getBodyByCustomInt(String propertyName, int valueToMatch) {
		Iterator<Body> iterator = m_bodiesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Body item = iterator.next();
			if (hasCustomInt(item, propertyName) && getCustomInt(item, propertyName, 0) == valueToMatch)
				return item;
		}
		return null;
	}

	Body getBodyByCustomFloat(String propertyName, float valueToMatch) {
		Iterator<Body> iterator = m_bodiesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Body item = iterator.next();
			if (hasCustomFloat(item, propertyName) && getCustomFloat(item, propertyName, 0) == valueToMatch)
				return item;
		}
		return null;
	}

	Body getBodyByCustomString(String propertyName, String valueToMatch) {
		Iterator<Body> iterator = m_bodiesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Body item = iterator.next();
			if (hasCustomString(item, propertyName) && getCustomString(item, propertyName, new String()).equals(valueToMatch))
				return item;
		}
		return null;
	}

	Body getBodyByCustomVector(String propertyName, Vector2 valueToMatch) {
		Iterator<Body> iterator = m_bodiesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Body item = iterator.next();
			if (hasCustomVector(item, propertyName) && getCustomVector(item, propertyName, new Vector2(0, 0)) == valueToMatch)
				return item;
		}
		return null;
	}

	Body getBodyByCustomBool(String propertyName, boolean valueToMatch) {
		Iterator<Body> iterator = m_bodiesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Body item = iterator.next();
			if (hasCustomBool(item, propertyName) && getCustomBool(item, propertyName, false) == valueToMatch)
				return item;
		}
		return null;
	}

	// get by custom property value (vector version, Fixture)
	public int getFixturesByCustomInt(String propertyName, int valueToMatch, Vector<Fixture> items) {
		Iterator<Fixture> iterator = m_fixturesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Fixture item = iterator.next();
			if (hasCustomInt(item, propertyName) && getCustomInt(item, propertyName, 0) == valueToMatch)
				items.add(item);
		}
		return items.size();
	}

	public int getFixturesByCustomFloat(String propertyName, float valueToMatch, Vector<Fixture> items) {
		Iterator<Fixture> iterator = m_fixturesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Fixture item = iterator.next();
			if (hasCustomFloat(item, propertyName) && getCustomFloat(item, propertyName, 0) == valueToMatch)
				items.add(item);
		}
		return items.size();
	}

	public int getFixturesByCustomString(String propertyName, String valueToMatch, Vector<Fixture> items) {
		Iterator<Fixture> iterator = m_fixturesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Fixture item = iterator.next();
			if (hasCustomString(item, propertyName) && getCustomString(item, propertyName, new String()).equals(valueToMatch))
				items.add(item);
		}
		return items.size();
	}

	public int getFixturesByCustomVector(String propertyName, Vector2 valueToMatch, Vector<Fixture> items) {
		Iterator<Fixture> iterator = m_fixturesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Fixture item = iterator.next();
			if (hasCustomVector(item, propertyName) && getCustomVector(item, propertyName, new Vector2(0, 0)) == valueToMatch)
				items.add(item);
		}
		return items.size();
	}

	public int getFixturesByCustomBool(String propertyName, boolean valueToMatch, Vector<Fixture> items) {
		Iterator<Fixture> iterator = m_fixturesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Fixture item = iterator.next();
			if (hasCustomBool(item, propertyName) && getCustomBool(item, propertyName, false) == valueToMatch)
				items.add(item);
		}
		return items.size();
	}

	// get by custom property value (single version, Fixture)
	Fixture getFixtureByCustomInt(String propertyName, int valueToMatch) {
		Iterator<Fixture> iterator = m_fixturesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Fixture item = iterator.next();
			if (hasCustomInt(item, propertyName) && getCustomInt(item, propertyName, 0) == valueToMatch)
				return item;
		}
		return null;
	}

	Fixture getFixtureByCustomFloat(String propertyName, float valueToMatch) {
		Iterator<Fixture> iterator = m_fixturesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Fixture item = iterator.next();
			if (hasCustomFloat(item, propertyName) && getCustomFloat(item, propertyName, 0) == valueToMatch)
				return item;
		}
		return null;
	}

	Fixture getFixtureByCustomString(String propertyName, String valueToMatch) {
		Iterator<Fixture> iterator = m_fixturesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Fixture item = iterator.next();
			if (hasCustomString(item, propertyName) && getCustomString(item, propertyName, new String()).equals(valueToMatch))
				return item;
		}
		return null;
	}

	Fixture getFixtureByCustomVector(String propertyName, Vector2 valueToMatch) {
		Iterator<Fixture> iterator = m_fixturesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Fixture item = iterator.next();
			if (hasCustomVector(item, propertyName) && getCustomVector(item, propertyName, new Vector2(0, 0)) == valueToMatch)
				return item;
		}
		return null;
	}

	Fixture getFixtureByCustomBool(String propertyName, boolean valueToMatch) {
		Iterator<Fixture> iterator = m_fixturesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Fixture item = iterator.next();
			if (hasCustomBool(item, propertyName) && getCustomBool(item, propertyName, false) == valueToMatch)
				return item;
		}
		return null;
	}

	// get by custom property value (vector version, Joint)
	public int getJointsByCustomInt(String propertyName, int valueToMatch, Vector<Joint> items) {
		Iterator<Joint> iterator = m_jointsWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Joint item = iterator.next();
			if (hasCustomInt(item, propertyName) && getCustomInt(item, propertyName, 0) == valueToMatch)
				items.add(item);
		}
		return items.size();
	}

	public int getJointsByCustomFloat(String propertyName, float valueToMatch, Vector<Joint> items) {
		Iterator<Joint> iterator = m_jointsWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Joint item = iterator.next();
			if (hasCustomFloat(item, propertyName) && getCustomFloat(item, propertyName, 0) == valueToMatch)
				items.add(item);
		}
		return items.size();
	}

	public int getJointsByCustomString(String propertyName, String valueToMatch, Vector<Joint> items) {
		Iterator<Joint> iterator = m_jointsWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Joint item = iterator.next();
			if (hasCustomString(item, propertyName) && getCustomString(item, propertyName, new String()).equals(valueToMatch))
				items.add(item);
		}
		return items.size();
	}

	public int getJointsByCustomVector(String propertyName, Vector2 valueToMatch, Vector<Joint> items) {
		Iterator<Joint> iterator = m_jointsWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Joint item = iterator.next();
			if (hasCustomVector(item, propertyName) && getCustomVector(item, propertyName, new Vector2(0, 0)) == valueToMatch)
				items.add(item);
		}
		return items.size();
	}

	public int getJointsByCustomBool(String propertyName, boolean valueToMatch, Vector<Joint> items) {
		Iterator<Joint> iterator = m_jointsWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Joint item = iterator.next();
			if (hasCustomBool(item, propertyName) && getCustomBool(item, propertyName, false) == valueToMatch)
				items.add(item);
		}
		return items.size();
	}

	// get by custom property value (single version, Joint)
	Joint getJointByCustomInt(String propertyName, int valueToMatch) {
		Iterator<Joint> iterator = m_jointsWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Joint item = iterator.next();
			if (hasCustomInt(item, propertyName) && getCustomInt(item, propertyName, 0) == valueToMatch)
				return item;
		}
		return null;
	}

	Joint getJointByCustomFloat(String propertyName, float valueToMatch) {
		Iterator<Joint> iterator = m_jointsWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Joint item = iterator.next();
			if (hasCustomFloat(item, propertyName) && getCustomFloat(item, propertyName, 0) == valueToMatch)
				return item;
		}
		return null;
	}

	Joint getJointByCustomString(String propertyName, String valueToMatch) {
		Iterator<Joint> iterator = m_jointsWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Joint item = iterator.next();
			if (hasCustomString(item, propertyName) && getCustomString(item, propertyName, new String()).equals(valueToMatch))
				return item;
		}
		return null;
	}

	Joint getJointByCustomVector(String propertyName, Vector2 valueToMatch) {
		Iterator<Joint> iterator = m_jointsWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Joint item = iterator.next();
			if (hasCustomVector(item, propertyName) && getCustomVector(item, propertyName, new Vector2(0, 0)) == valueToMatch)
				return item;
		}
		return null;
	}

	Joint getJointByCustomBool(String propertyName, boolean valueToMatch) {
		Iterator<Joint> iterator = m_jointsWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Joint item = iterator.next();
			if (hasCustomBool(item, propertyName) && getCustomBool(item, propertyName, false) == valueToMatch)
				return item;
		}
		return null;
	}

	// get by custom property value (vector version, Image)
	public int getImagesByCustomInt(String propertyName, int valueToMatch, Vector<Jb2dJsonImage> items) {
		Iterator<Jb2dJsonImage> iterator = m_imagesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Jb2dJsonImage item = iterator.next();
			if (hasCustomInt(item, propertyName) && getCustomInt(item, propertyName, 0) == valueToMatch)
				items.add(item);
		}
		return items.size();
	}

	public int getImagesByCustomFloat(String propertyName, float valueToMatch, Vector<Jb2dJsonImage> items) {
		Iterator<Jb2dJsonImage> iterator = m_imagesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Jb2dJsonImage item = iterator.next();
			if (hasCustomFloat(item, propertyName) && getCustomFloat(item, propertyName, 0) == valueToMatch)
				items.add(item);
		}
		return items.size();
	}

	public int getImagesByCustomString(String propertyName, String valueToMatch, Vector<Jb2dJsonImage> items) {
		Iterator<Jb2dJsonImage> iterator = m_imagesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Jb2dJsonImage item = iterator.next();
			if (hasCustomString(item, propertyName) && getCustomString(item, propertyName, new String()).equals(valueToMatch))
				items.add(item);
		}
		return items.size();
	}

	public int getImagesByCustomVector(String propertyName, Vector2 valueToMatch, Vector<Jb2dJsonImage> items) {
		Iterator<Jb2dJsonImage> iterator = m_imagesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Jb2dJsonImage item = iterator.next();
			if (hasCustomVector(item, propertyName) && getCustomVector(item, propertyName, new Vector2(0, 0)) == valueToMatch)
				items.add(item);
		}
		return items.size();
	}

	public int getImagesByCustomBool(String propertyName, boolean valueToMatch, Vector<Jb2dJsonImage> items) {
		Iterator<Jb2dJsonImage> iterator = m_imagesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Jb2dJsonImage item = iterator.next();
			if (hasCustomBool(item, propertyName) && getCustomBool(item, propertyName, false) == valueToMatch)
				items.add(item);
		}
		return items.size();
	}

	// get by custom property value (single version, Image)
	Jb2dJsonImage getImageByCustomInt(String propertyName, int valueToMatch) {
		Iterator<Jb2dJsonImage> iterator = m_imagesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Jb2dJsonImage item = iterator.next();
			if (hasCustomInt(item, propertyName) && getCustomInt(item, propertyName, 0) == valueToMatch)
				return item;
		}
		return null;
	}

	Jb2dJsonImage getImageByCustomFloat(String propertyName, float valueToMatch) {
		Iterator<Jb2dJsonImage> iterator = m_imagesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Jb2dJsonImage item = iterator.next();
			if (hasCustomFloat(item, propertyName) && getCustomFloat(item, propertyName, 0) == valueToMatch)
				return item;
		}
		return null;
	}

	Jb2dJsonImage getImageByCustomString(String propertyName, String valueToMatch) {
		Iterator<Jb2dJsonImage> iterator = m_imagesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Jb2dJsonImage item = iterator.next();
			if (hasCustomString(item, propertyName) && getCustomString(item, propertyName, new String()).equals(valueToMatch))
				return item;
		}
		return null;
	}

	Jb2dJsonImage getImageByCustomVector(String propertyName, Vector2 valueToMatch) {
		Iterator<Jb2dJsonImage> iterator = m_imagesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Jb2dJsonImage item = iterator.next();
			if (hasCustomVector(item, propertyName) && getCustomVector(item, propertyName, new Vector2(0, 0)) == valueToMatch)
				return item;
		}
		return null;
	}

	Jb2dJsonImage getImageByCustomBool(String propertyName, boolean valueToMatch) {
		Iterator<Jb2dJsonImage> iterator = m_imagesWithCustomProperties.iterator();
		while (iterator.hasNext()) {
			Jb2dJsonImage item = iterator.next();
			if (hasCustomBool(item, propertyName) && getCustomBool(item, propertyName, false) == valueToMatch)
				return item;
		}
		return null;
	}

	protected JSONArray writeCustomPropertiesToJson(Object item) throws JSONException {
		JSONArray customPropertiesValue = new JSONArray();

		Jb2dJsonCustomProperties props = getCustomPropertiesForItem(item, false);
		if (null == props)
			return customPropertiesValue;

		int i = 0;

		{
			Iterator<ObjectMap.Entry<String, Integer>> it = props.m_customPropertyMap_int.entries().iterator();
			while (it.hasNext()) {
				ObjectMap.Entry<String, Integer> pair = (ObjectMap.Entry<String, Integer>) it.next();
				JSONObject propValue = new JSONObject();
				propValue.put("name", pair.key);
				propValue.put("int", pair.value);
				customPropertiesValue.put(i++, propValue);
			}
		}
		{
			Iterator<ObjectMap.Entry<String, Double>> it = props.m_customPropertyMap_float.entries().iterator();
			while (it.hasNext()) {
				ObjectMap.Entry<String, Double> pair = (ObjectMap.Entry<String, Double>) it.next();
				JSONObject propValue = new JSONObject();
				propValue.put("name", pair.key);
				propValue.put("float", pair.value);
				customPropertiesValue.put(i++, propValue);
			}
		}
		{
			Iterator<ObjectMap.Entry<String, String>> it = props.m_customPropertyMap_string.entries().iterator();
			while (it.hasNext()) {
				ObjectMap.Entry<String, String> pair = (ObjectMap.Entry<String, String>) it.next();
				JSONObject propValue = new JSONObject();
				propValue.put("name", pair.key);
				propValue.put("string", pair.value);
				customPropertiesValue.put(i++, propValue);
			}
		}
		{
			Iterator<ObjectMap.Entry<String, Vector2>> it = props.m_customPropertyMap_vec2.entries().iterator();
			while (it.hasNext()) {
				ObjectMap.Entry<String, Vector2> pair = (ObjectMap.Entry<String, Vector2>) it.next();
				JSONObject propValue = new JSONObject();
				propValue.put("name", pair.key);
				vecToJson("Vector2", pair.value, propValue);
				customPropertiesValue.put(i++, propValue);
			}
		}
		{
			Iterator<ObjectMap.Entry<String, Boolean>> it = props.m_customPropertyMap_bool.entries().iterator();
			while (it.hasNext()) {
				ObjectMap.Entry<String, Boolean> pair = (ObjectMap.Entry<String, Boolean>) it.next();
				JSONObject propValue = new JSONObject();
				propValue.put("name", pair.key);
				propValue.put("bool", pair.value);
				customPropertiesValue.put(i++, propValue);
			}
		}

		return customPropertiesValue;
	}

	protected void readCustomPropertiesFromJson(Body item, JSONObject value) throws JSONException {
		if (null == item)
			return;

		if (!value.has("customProperties"))
			return;

		int i = 0;
		JSONArray propValues = value.getJSONArray("customProperties");
		if (null != propValues) {
			int numPropValues = propValues.length();
			for (i = 0; i < numPropValues; i++) {
				JSONObject propValue = propValues.getJSONObject(i);
				String propertyName = propValue.getString("name");
				if (propValue.has("int"))
					setCustomInt(item, propertyName, propValue.getInt("int"));
				if (propValue.has("float"))
					setCustomFloat(item, propertyName, (float) propValue.getDouble("float"));
				if (propValue.has("string"))
					setCustomString(item, propertyName, propValue.getString("string"));
				if (propValue.has("vec2"))
					setCustomVector(item, propertyName, this.jsonToVec("vec2", propValue));
				if (propValue.has("bool"))
					setCustomBool(item, propertyName, propValue.getBoolean("bool"));
			}
		}
	}


	protected void readCustomPropertiesFromJson(Fixture item, JSONObject value) throws JSONException {
		if (null == item)
			return;

		if (!value.has("customProperties"))
			return;

		int i = 0;
		JSONArray propValues = value.getJSONArray("customProperties");
		if (null != propValues) {
			int numPropValues = propValues.length();
			for (i = 0; i < numPropValues; i++) {
				JSONObject propValue = propValues.getJSONObject(i);
				String propertyName = propValue.getString("name");
				if (propValue.has("int"))
					setCustomInt(item, propertyName, propValue.getInt("int"));
				if (propValue.has("float"))
					setCustomFloat(item, propertyName, (float) propValue.getDouble("float"));
				if (propValue.has("string"))
					setCustomString(item, propertyName, propValue.getString("string"));
				if (propValue.has("vec2"))
					setCustomVector(item, propertyName, this.jsonToVec("vec2", propValue));
				if (propValue.has("bool"))
					setCustomBool(item, propertyName, propValue.getBoolean("bool"));
			}
		}
	}

	protected void readCustomPropertiesFromJson(Joint item, JSONObject value) throws JSONException {
		if (null == item)
			return;

		if (!value.has("customProperties"))
			return;

		int i = 0;
		JSONArray propValues = value.getJSONArray("customProperties");
		if (null != propValues) {
			int numPropValues = propValues.length();
			for (i = 0; i < numPropValues; i++) {
				JSONObject propValue = propValues.getJSONObject(i);
				String propertyName = propValue.getString("name");
				if (propValue.has("int"))
					setCustomInt(item, propertyName, propValue.getInt("int"));
				if (propValue.has("float"))
					setCustomFloat(item, propertyName, (float) propValue.getDouble("float"));
				if (propValue.has("string"))
					setCustomString(item, propertyName, propValue.getString("string"));
				if (propValue.has("vec2"))
					setCustomVector(item, propertyName, this.jsonToVec("vec2", propValue));
				if (propValue.has("bool"))
					setCustomBool(item, propertyName, propValue.getBoolean("bool"));
			}
		}
	}

	protected void readCustomPropertiesFromJson(Jb2dJsonImage item, JSONObject value) throws JSONException {
		if (null == item)
			return;

		if (!value.has("customProperties"))
			return;

		int i = 0;
		JSONArray propValues = value.getJSONArray("customProperties");
		if (null != propValues) {
			int numPropValues = propValues.length();
			for (i = 0; i < numPropValues; i++) {
				JSONObject propValue = propValues.getJSONObject(i);
				String propertyName = propValue.getString("name");
				if (propValue.has("int"))
					setCustomInt(item, propertyName, propValue.getInt("int"));
				if (propValue.has("float"))
					setCustomFloat(item, propertyName, (float) propValue.getDouble("float"));
				if (propValue.has("string"))
					setCustomString(item, propertyName, propValue.getString("string"));
				if (propValue.has("vec2"))
					setCustomVector(item, propertyName, this.jsonToVec("vec2", propValue));
				if (propValue.has("bool"))
					setCustomBool(item, propertyName, propValue.getBoolean("bool"));
			}
		}
	}

	protected void readCustomPropertiesFromJson(World item, JSONObject value) throws JSONException {
		if (null == item)
			return;

		if (!value.has("customProperties"))
			return;

		int i = 0;
		JSONArray propValues = value.getJSONArray("customProperties");
		if (null != propValues) {
			int numPropValues = propValues.length();
			for (i = 0; i < numPropValues; i++) {
				JSONObject propValue = propValues.getJSONObject(i);
				String propertyName = propValue.getString("name");
				if (propValue.has("int"))
					setCustomInt(item, propertyName, propValue.getInt("int"));
				if (propValue.has("float"))
					setCustomFloat(item, propertyName, (float) propValue.getDouble("float"));
				if (propValue.has("string"))
					setCustomString(item, propertyName, propValue.getString("string"));
				if (propValue.has("vec2"))
					setCustomVector(item, propertyName, this.jsonToVec("vec2", propValue));
				if (propValue.has("bool"))
					setCustomBool(item, propertyName, propValue.getBoolean("bool"));
			}
		}
	}

}
