package extendModelRule;

import java.util.ArrayList;
import java.util.List;

import com.jme3.math.Vector3f;

import eu.opends.basics.MapObject;
import eu.opends.canbus.CANClient;
import eu.opends.car.SteeringCar;
import eu.opends.environment.TrafficLight;
import eu.opends.environment.TrafficLight.TrafficLightState;
import eu.opends.main.Simulator;
import eu.opends.traffic.PhysicalTraffic;
import eu.opends.traffic.TrafficObject;

public class ModelRule {
	private final float DefaultAccel = -1f;
	private final float DefaultSpeed = 50f;
	private final float SavingSpeed = 30f;
	private final float CautionSpeed = 15f;
	private final float CurveSpeed = 15f;
	private final float SensorRangeForward = 20f;
	private final float SensorRangeSide = 3f;
	private final float SensorRangeBehind = 10f;
	private final float SensorRangeEmergency = 10f;
	private final float SensorRangeCarWidth = 1.5f;
	private final float HittingDistance = 3f;
	private final boolean rotation = false;
	private final boolean ondebug = true;
	private final boolean auto = true;
	private boolean ondebugFlame = false;
	private boolean inGoal = false;
	private float objectiveSpeed;
	private float steeringValue = 0;
	private int flameCount = 0;
	private Simulator sim;
	private SteeringCar car;
	private List<MapObject> crossingPointList;
	private MapObject[] route;
	private String[] routeName = { "01", "02", "05", "06", "07", "08", "09", "10", "11", "12", "13", "14",
			"15", "22" };
	private int currentRoute = 0;
	private int RouteSize;

	public ModelRule(Simulator sim, SteeringCar car, List<MapObject> modelList) {
		this.sim = sim;
		this.car = car;
		this.crossingPointList = new ArrayList<MapObject>();
		for (MapObject model : modelList) {
			if (model.getName().contains("point")) {
				this.crossingPointList.add(model);
			}
		}
		this.route = new MapObject[100];
		for (String routeName : this.routeName) {
			for (MapObject crossingPoint : crossingPointList) {
				if (crossingPoint.getName().contains(routeName)) {
					route[currentRoute++] = crossingPoint;
				}
			}
		}
		RouteSize = currentRoute;
		currentRoute = 0;
	}

	public void activateModelRule() {
		if (ondebug) {
			ondebugFlame = false;
			flameCount++;
			if (flameCount == 10) {
				System.out.println("current car position = " + car.getPosition());
				System.out.println("current objective speed = " + objectiveSpeed);
				System.out.println("current car direction = " + getCarDirection());
				double alpha = Math.toRadians(getCarDirection());
				double pivotX = car.getPosition().getX() - Math.sin(alpha) * car.getCurrentSpeedKmh() / 2;
				double pivotZ = car.getPosition().getZ() - Math.cos(alpha) * car.getCurrentSpeedKmh() / 2;
				System.out.println("Sensor pivot = (" + pivotX + "," + pivotZ + ")");
				flameCount = 0;
				ondebugFlame = true;
			}
		}
		MapObject hittingPoint;
		if (auto) {
			if (precedingCarSensor2(true) || pedestrianSensor2(true) || trafficLightSensor2(true)) {
				emergencyRule();
			} else if (precedingCarSensor2(false) || pedestrianSensor2(false) || trafficLightSensor2(false)) {
				cautionRule();
			} else {
				defaultRule();
			}
			if (inGoal) {
				inGoalRule();
			} else if ((hittingPoint = hittingNaviPointSensor()) != null) {
				hittingNaviPointRule(hittingPoint);
			} else if (crossingPointSensor2(false)) {
				crossingPointRule();
			}
			steeringToTarget(route[currentRoute].getLocation());
			setSpeed(objectiveSpeed);
		}
	}

	/* 自車中心のセンサー(現在未使用) */
	private double[] getSensorPoint(int index) {
		double x = car.getPosition().getX();
		double z = car.getPosition().getZ();
		double alpha = Math.toRadians(getCarDirection());
		if (index <= 4) {
			x -= Math.sin(alpha) * SensorRangeForward;
			z -= Math.cos(alpha) * SensorRangeForward;
		} else if (index <= 9) {
			x -= Math.sin(alpha) * SensorRangeEmergency;
			z -= Math.cos(alpha) * SensorRangeEmergency;
		} else if (index >= 15) {
			x += Math.sin(alpha) * SensorRangeBehind;
			z += Math.cos(alpha) * SensorRangeBehind;
		}
		switch (index % 5) {
		case 0:
			x -= Math.sin(alpha + Math.PI / 2) * SensorRangeSide;
			z -= Math.cos(alpha + Math.PI / 2) * SensorRangeSide;
			break;
		case 1:
			x -= Math.sin(alpha + Math.PI / 2) * SensorRangeCarWidth;
			z -= Math.cos(alpha + Math.PI / 2) * SensorRangeCarWidth;
			break;
		case 3:
			x -= Math.sin(alpha - Math.PI / 2) * SensorRangeCarWidth;
			z -= Math.cos(alpha - Math.PI / 2) * SensorRangeCarWidth;
			break;
		case 4:
			x -= Math.sin(alpha - Math.PI / 2) * SensorRangeSide;
			z -= Math.cos(alpha - Math.PI / 2) * SensorRangeSide;
			break;
		default:
			break;
		}
		double[] pair = { x, z };
		return pair;
	}

	private boolean findTarget(Vector3f target, int[] pointIndex) {
		double dx[] = new double[5];
		double dz[] = new double[5];
		for (int i = 0; i < 4; i++) {
			double[] pair = getSensorPoint(pointIndex[i]);
			dx[i] = pair[0] - target.getX();
			dz[i] = pair[1] - target.getZ();
		}
		dx[4] = dx[0];
		dz[4] = dz[0];
		double angle = 0;
		for (int i = 0; i < 4; i++) {
			angle += Math.atan2(dx[i] * dz[i + 1] - dz[i] * dx[i + 1], dx[i] * dx[i + 1] + dz[i] * dz[i + 1]);
		}
		if (2 * Math.PI <= angle + 0.00001 && 2 * Math.PI >= angle - 0.00001) {
			return true;
		}
		return false;
	}

	private boolean precedingCarSensor(boolean inEmergency) {
		int[] sensorPoints = { inEmergency ? 6 : 1, inEmergency ? 8 : 3, 13, 11 };
		for (TrafficObject car : PhysicalTraffic.getTrafficObjectList()) {
			if (car.getName().contains("car")) {
				if (findTarget(car.getPosition(), sensorPoints)) {
					if (ondebugFlame) {
						System.out.println(inEmergency ? "find preceding car in emergency" : "find preceding car");
					}
					return true;
				}
			}
		}
		return false;
	}

	private boolean pedestrianSensor(boolean inEmergency) {
		int[] sensorPoints = { inEmergency ? 6 : 0, inEmergency ? 8 : 4, inEmergency ? 13 : 14, inEmergency ? 11 : 10 };
		for (TrafficObject pedestrian : PhysicalTraffic.getTrafficObjectList()) {
			if (pedestrian.getName().contains("pedestrian")) {
				if (findTarget(pedestrian.getPosition(), sensorPoints)) {
					if (ondebugFlame) {
						System.out.println(inEmergency ? "find predestrian in emergency" : "find predestrian");
					}
					return true;
				}
			}
		}
		return false;
	}

	private boolean trafficLightSensor(boolean inEmergency) {
		int[] sensorPoints = { inEmergency ? 5 : 0, inEmergency ? 9 : 4, 14, 10 };
		for (TrafficLight trafficLight : sim.getTrafficLightCenter().getGrobalTrafficLightList()) {
			if (findTarget(trafficLight.getLocalPosition(), sensorPoints)) {
				double direction = Math.toDegrees(trafficLight.getRotation().toAngleAxis(new Vector3f(0, 1, 0)));
				direction = trafficLight.getRotation().getY() > 0 ? direction : -direction;
				double carDirection = getCarDirection();
				if ((direction < carDirection + 5 && direction > carDirection - 5) || direction < carDirection - 355
						|| direction > carDirection + 355) {
					if (trafficLight.getState() != TrafficLightState.GREEN) {
						if (ondebugFlame) {
							System.out.println(inEmergency ? "find traffic light in emergency" : "find traffic light");
						}
						return true;
					}
				}
			}
		}
		return false;
	}

	private boolean crossingPointSensor(boolean inEmergency) {
		int[] sensorPoints = { inEmergency ? 6 : 1, inEmergency ? 8 : 3, 18, 16 };
		for (MapObject crossingPoint : crossingPointList) {
			if (findTarget(crossingPoint.getLocation(), sensorPoints)) {
				if (ondebugFlame) {
					System.out.println("find crossing point[" + crossingPoint.getName() + "]");
				}
				return true;
			}
		}
		return false;
	}
	/* */

	/* 先読み型センサー */
	private SensorPointSet getSensorPoint2(double left, double right, double forward, double behind) {
		double alpha = Math.toRadians(getCarDirection());
		double pivotX = car.getPosition().getX() - Math.sin(alpha) * car.getCurrentSpeedKmh() / 3;
		double pivotZ = car.getPosition().getZ() - Math.cos(alpha) * car.getCurrentSpeedKmh() / 3;
		if(steeringValue > 0){
			behind += steeringValue * 30;
		} else if (steeringValue < 0) {
			behind -= steeringValue * 30;
		}
		SensorPointSet sps = new SensorPointSet();
		sps.addPoint(pivotX - Math.sin(alpha) * forward - Math.sin(alpha + Math.PI / 2) * left,
				pivotZ - Math.cos(alpha) * forward - Math.cos(alpha + Math.PI / 2) * left);
		sps.addPoint(pivotX - Math.sin(alpha) * forward - Math.sin(alpha - Math.PI / 2) * right,
				pivotZ - Math.cos(alpha) * forward - Math.cos(alpha - Math.PI / 2) * right);
		sps.addPoint(pivotX - Math.sin(alpha + Math.PI) * behind - Math.sin(alpha - Math.PI / 2) * right,
				pivotZ - Math.cos(alpha + Math.PI) * behind - Math.cos(alpha - Math.PI / 2) * right);
		sps.addPoint(pivotX - Math.sin(alpha + Math.PI) * behind - Math.sin(alpha + Math.PI / 2) * left,
				pivotZ - Math.cos(alpha + Math.PI) * behind - Math.cos(alpha + Math.PI / 2) * left);
		return sps;
	}

	class SensorPointSet {
		double x[];
		double z[];
		int size = 0;

		public SensorPointSet() {
			x = new double[4];
			z = new double[4];
		}

		public void addPoint(double x, double z) {
			this.x[size] = x;
			this.z[size] = z;
			size++;
		}

		public double[] getdx(Vector3f target) {
			double dx[] = new double[size + 1];
			for (int i = 0; i < size; i++) {
				dx[i] = x[i] - target.getX();
			}
			dx[size] = dx[0];
			return dx;
		}

		public double[] getdz(Vector3f target) {
			double dz[] = new double[size + 1];
			for (int i = 0; i < size; i++) {
				dz[i] = z[i] - target.getZ();
			}
			dz[size] = dz[0];
			return dz;
		}
	}

	private boolean findTarget2(Vector3f target, double left, double right, double forward, double behind,
			boolean ondebug) {
		SensorPointSet sps = getSensorPoint2(left, right, forward, behind);
		double angle = 0;
		double dx[] = sps.getdx(target);
		double dz[] = sps.getdz(target);
		for(int i = 0;i<4;i++){
			angle += Math.atan2(dx[i] * dz[i + 1] - dz[i] * dx[i + 1], dx[i] * dx[i + 1] + dz[i] * dz[i + 1]);
		}
		if (2 * Math.PI <= angle + 0.00001 && 2 * Math.PI >= angle - 0.00001) {
			for (int i = 0; i < 4; i++) {
				if (ondebug) {
					System.out.println("sensorX[" + i + "]" + sps.x[i]);
					System.out.println("sensorZ[" + i + "]" + sps.z[i]);
				}
			}
			return true;
		}
		return false;
	}

	private boolean precedingCarSensor2(boolean inEmergency) {
		float left = SensorRangeCarWidth;
		float right = SensorRangeCarWidth;
		float forward = inEmergency ? SensorRangeEmergency : SensorRangeForward;
		float behind = 0;
		for (TrafficObject car : PhysicalTraffic.getTrafficObjectList()) {
			if (car.getName().contains("car")) {
				if (findTarget2(car.getPosition(), left, right, forward, behind, ondebugFlame)) {
					if (ondebugFlame) {
						System.out.println(inEmergency ? "find preceding car in emergency" : "find preceding car");
					}
					return true;
				}
			}
		}
		return false;
	}



	private boolean pedestrianSensor2(boolean inEmergency) {
		float left = SensorRangeSide;
		float right = SensorRangeCarWidth;
		float forward = inEmergency ? SensorRangeEmergency : SensorRangeForward;
		float behind = 0;
		for (TrafficObject pedestrian : PhysicalTraffic.getTrafficObjectList()) {
			if (pedestrian.getName().contains("pedestrian")) {
				if (findTarget2(pedestrian.getPosition(), left, right, forward, behind, ondebugFlame)) {
					if (ondebugFlame) {
						System.out.println(inEmergency ? "find predestrian in emergency" : "find predestrian");
					}
					return true;
				}
			}
		}
		return false;
	}



	private boolean trafficLightSensor2(boolean inEmergency) {
		float left = SensorRangeSide;
		float right = SensorRangeSide;
		float forward = inEmergency ? SensorRangeEmergency : SensorRangeForward;
		float behind = 0;
		for (TrafficLight trafficLight : sim.getTrafficLightCenter().getGrobalTrafficLightList()) {
			if (findTarget2(trafficLight.getLocalPosition(), left, right, forward, behind, ondebugFlame)) {
				double direction = Math.toDegrees(trafficLight.getRotation().toAngleAxis(new Vector3f(0, 1, 0)));
				direction = trafficLight.getRotation().getY() > 0 ? direction : -direction;
				double carDirection = getCarDirection();
				if((direction < carDirection + 5 && direction > carDirection - 5) || direction < carDirection -355 || direction > carDirection + 355){
					if (trafficLight.getState() != TrafficLightState.GREEN) {
						if (ondebugFlame) {
							System.out.println(inEmergency ? "find traffic light in emergency" : "find traffic light");
						}
						return true;
					}
				}
			}
		}
		return false;
	}

	private boolean crossingPointSensor2(boolean inEmergency) {
		float left = SensorRangeCarWidth;
		float right = SensorRangeCarWidth;
		float forward = inEmergency ? SensorRangeEmergency : SensorRangeForward;
		float behind = 0;
		for (MapObject crossingPoint : crossingPointList) {
			if (findTarget2(crossingPoint.getLocation(), left, right, forward, behind, false)) {
				if (ondebugFlame) {
					System.out.println("find crossing point[" + crossingPoint.getName() + "]");
				}
				return true;
			}
		}
		return false;
	}

	private MapObject hittingNaviPointSensor() {
		for (MapObject crossingPoint : crossingPointList) {
			if (hittingNaviPoint(crossingPoint.getLocation())) {
				if (ondebugFlame) {
					System.out.println("hit crossing point[" + crossingPoint.getName() + "]");
				}
				return crossingPoint;
			}
		}
		return null;
	}

	private boolean hittingNaviPoint(Vector3f target) {
		return this.car.getPosition().distance(target) < HittingDistance;
	}
	/* */

	/* 実行ルール */
	private void emergencyRule() {
		objectiveSpeed = 0;
	}

	private void cautionRule() {
		objectiveSpeed = CautionSpeed;
	}

	private void defaultRule() {
		objectiveSpeed = DefaultSpeed;
	}

	private void inGoalRule() {
		objectiveSpeed = 0;
	}

	private void crossingPointRule() {
		objectiveSpeed = objectiveSpeed > SavingSpeed ? SavingSpeed : objectiveSpeed;
	}

	private void hittingNaviPointRule(MapObject hittingPoint) {
		if (hittingPoint == route[currentRoute]) {
			objectiveSpeed = objectiveSpeed > CurveSpeed ? CurveSpeed : objectiveSpeed;
			currentRoute++;
			if (currentRoute == RouteSize) {
				if (rotation) {
					currentRoute = 0;
				} else {
					currentRoute = 0;
					inGoal = true;
				}
			}
		}
		if (currentRoute > 0 && hittingPoint == route[currentRoute - 1]) {
			objectiveSpeed = objectiveSpeed > CurveSpeed ? CurveSpeed : objectiveSpeed;
		}
	}
	/* */

	/* 自動操作 */
	private double getTargetDirection(Vector3f target) {
		return Math.toDegrees(Math.atan2(this.car.getPosition().getX() - target.getX(),
				this.car.getPosition().getZ() - target.getZ()));
	}

	private double getCarDirection() {
		double direction = Math.toDegrees(car.getRotation().toAngleAxis(new Vector3f(0, 1, 0)));
		return car.getRotation().getY() > 0 ? direction : -direction;
	}

	private double getSteeringValue(Vector3f target) {
		double handLink = getTargetDirection(target) - getCarDirection();
		if (handLink < 1 && handLink > -1)
			return 0;
		handLink = handLink > 180 ? handLink - 360 : handLink;
		handLink = handLink < -180 ? 360 + handLink : handLink;
		handLink = (handLink > 0 ? Math.log10(handLink) : -Math.log10(-handLink)) / 4;
		return handLink;
	}

	private void setSpeed(float objectiveSpeed) {
		sim.getSteeringTask().getPrimaryTask().reportGreenLight();
		float accelerationValue;
		if (objectiveSpeed == 0) {
			car.setBrakePedalIntensity(1f);
			sim.getThreeVehiclePlatoonTask().reportBrakeIntensity(1f);
			car.disableCruiseControlByBrake();
			accelerationValue = 0;
		} else {
			accelerationValue = car.getCurrentSpeedKmh() < objectiveSpeed ? this.DefaultAccel : -this.DefaultAccel;
		}
		sim.getThreeVehiclePlatoonTask().reportAcceleratorIntensity(Math.abs(accelerationValue));
		car.setAcceleratorPedalIntensity(accelerationValue);
	}

	private void steeringToTarget(Vector3f target) {
		steeringValue = (float) getSteeringValue(target);
		if (ondebugFlame) {
			System.out.println("current steering value = " + steeringValue);
		}
		CANClient canClient = Simulator.getCanClient();
		if (canClient != null)
			canClient.suppressSteering();
		sim.getSteeringTask().setSteeringIntensity(-3 * steeringValue);
		car.steer(steeringValue);
	}
	/* */
}
