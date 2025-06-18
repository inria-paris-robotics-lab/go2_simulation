import numpy as np
from go2_description.loader import loadGo2
import hppfcl
import pinocchio as pin
import simple
from go2_simulation.abstract_wrapper import AbstractSimulatorWrapper

class SimpleSimulator:
    def __init__(self, model, geom_model, visual_model, q0, args):
        self.model = model
        self.geom_model = geom_model
        self.visual_model = visual_model
        self.args = args

        self.data = self.model.createData()
        self.geom_data = self.geom_model.createData()

        for col_req in self.geom_data.collisionRequests:
            col_req: hppfcl.CollisionRequest
            col_req.security_margin = 0.0
            col_req.break_distance = 0.0
            col_req.gjk_tolerance = 1e-6
            col_req.epa_tolerance = 1e-6
            col_req.gjk_initial_guess = hppfcl.GJKInitialGuess.CachedGuess
            col_req.gjk_variant = hppfcl.GJKVariant.DefaultGJK

        for patch_req in self.geom_data.contactPatchRequests:
            patch_req.setPatchTolerance(args["patch_tolerance"])

        # Simulation parameters
        self.simulator = simple.Simulator(model, self.data, geom_model, self.geom_data)
        self.simulator.contact_solver_info = pin.ProximalSettings(
            args["tol"], args["tol_rel"], args["mu_prox"], args["maxit"]
        )
        #
        self.simulator.warm_start_constraint_forces = args["warm_start"]
        self.simulator.measure_timings = True
        # Contact patch settings
        self.simulator.constraints_problem.setMaxNumberOfContactsPerCollisionPair(
            args["max_patch_size"]
        )
        # Baumgarte settings
        self.simulator.constraints_problem.Kp = args["Kp"]
        self.simulator.constraints_problem.Kd = args["Kd"]
        if args["admm_update_rule"] == "spectral":
            self.simulator.admm_constraint_solver_settings.admm_update_rule = (
                pin.ADMMUpdateRule.SPECTRAL
            )
        elif args["admm_update_rule"] == "linear":
            self.simulator.admm_constraint_solver_settings.admm_update_rule = (
                pin.ADMMUpdateRule.LINEAR
            )
        else:
            update_rule = args["admm_update_rule"]
            print(f"ERROR - no match for admm update rule {update_rule}")
            exit(1)
        self.dt = args["dt"]

        # Initialize robot state
        self.q = q0.copy()
        self.v = np.zeros(self.model.nv)
        self.a = np.zeros(self.model.nv)
        self.f_feet = np.zeros(4)
        self.fext = [pin.Force(np.zeros(6)) for _ in range(self.model.njoints)]

        fps = min([self.args["max_fps"], 1.0 / self.dt])
        self.dt_vis = 1.0 / float(fps)
        self.simulator.reset()


    def execute(self, tau):
        if self.args["contact_solver"] == "ADMM":
            self.simulator.step(self.q, self.v, tau, self.fext, self.dt)
        else:
            self.simulator.stepPGS(self.q, self.v, tau, self.fext, self.dt)
        #print(self.simulator.getStepCPUTimes().user)
        self.q = self.simulator.qnew.copy()
        self.v = self.simulator.vnew.copy()
        self.a = self.simulator.anew.copy()

        #print("elapsed simu time " + str(step_end - step_start))
        #time_until_next_step = self.dt_vis - (time.time() - step_start)
        #if time_until_next_step > 0:
        #    time.sleep(time_until_next_step)

        return self.q, self.v, self.a, self.f_feet

    def view_state(self, q):
        self.vizer.display(q)


def setPhysicsProperties(
    geom_model: pin.GeometryModel, material: str, compliance: float
):
    for gobj in geom_model.geometryObjects:
        if material == "ice":
            gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.ICE
        elif material == "plastic":
            gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.PLASTIC
        elif material == "wood":
            gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.WOOD
        elif material == "metal":
            gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.METAL
        elif material == "concrete":
            gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.CONCRETE

        # Compliance
        gobj.physicsMaterial.compliance = compliance


def removeBVHModelsIfAny(geom_model: pin.GeometryModel):
    for gobj in geom_model.geometryObjects:
        gobj: pin.GeometryObject
        bvh_types = [hppfcl.BV_OBBRSS, hppfcl.BV_OBB, hppfcl.BV_AABB]
        ntype = gobj.geometry.getNodeType()
        if ntype in bvh_types:
            gobj.geometry.buildConvexHull(True, "Qt")
            gobj.geometry = gobj.geometry.convex


def addFloor(geom_model: pin.GeometryModel, visual_model: pin.GeometryModel):
    # Collision object
    # floor_collision_shape = hppfcl.Box(10, 10, 2)
    # M = pin.SE3(np.eye(3), np.zeros(3))
    # M.translation = np.array([0.0, 0.0, -(1.99 / 2.0)])
    floor_collision_shape = hppfcl.Halfspace(0, 0, 1, 0)
    # floor_collision_shape = hppfcl.Plane(0, 0, 1, 0)
    # floor_collision_shape.setSweptSphereRadius(0.5)
    M = pin.SE3.Identity()
    floor_collision_object = pin.GeometryObject("floor", 0, 0, M, floor_collision_shape)
    geom_model.addGeometryObject(floor_collision_object)

    # Visual object
    floor_visual_shape = hppfcl.Box(10, 10, 0.01)
    floor_visual_object = pin.GeometryObject(
        "floor", 0, 0, pin.SE3.Identity(), floor_visual_shape
    )
    visual_model.addGeometryObject(floor_visual_object)

def addSystemCollisionPairs(model, geom_model, qref):
    """
    Add the right collision pairs of a model, given qref.
    qref is here as a `T-pose`. The function uses this pose to determine which objects are in collision
    in this ref pose. If objects are in collision, they are not added as collision pairs, as they are considered
    to always be in collision.
    """
    data = model.createData()
    geom_data = geom_model.createData()
    pin.updateGeometryPlacements(model, data, geom_model, geom_data, qref)
    geom_model.removeAllCollisionPairs()
    num_col_pairs = 0
    for i in range(len(geom_model.geometryObjects)):
        for j in range(i+1, len(geom_model.geometryObjects)):
            # Don't add collision pair if same object
            if i != j:
                gobj_i: pin.GeometryObject = geom_model.geometryObjects[i]
                gobj_j: pin.GeometryObject = geom_model.geometryObjects[j]
                if gobj_i.name == "floor" or gobj_j.name == "floor":
                    num_col_pairs += 1
                    col_pair = pin.CollisionPair(i, j)
                    geom_model.addCollisionPair(col_pair)
                else:
                    if gobj_i.parentJoint != gobj_j.parentJoint or gobj_i.parentJoint == 0:
                        if gobj_i.parentJoint != model.parents[gobj_j.parentJoint] and gobj_j.parentJoint != model.parents[gobj_i.parentJoint] or gobj_i.parentJoint == 0 or gobj_j.parentJoint == 0:
                            # Compute collision between the geometries. Only add the collision pair if there is no collision.
                            M1 = geom_data.oMg[i]
                            M2 = geom_data.oMg[j]
                            colreq = hppfcl.CollisionRequest()
                            colreq.security_margin = 1e-2 # 1cm of clearance
                            colres = hppfcl.CollisionResult()
                            hppfcl.collide(gobj_i.geometry, M1, gobj_j.geometry, M2, colreq, colres)
                            if not colres.isCollision():
                                num_col_pairs += 1
                                col_pair = pin.CollisionPair(i, j)
                                geom_model.addCollisionPair(col_pair)
    print("Num col pairs = ", num_col_pairs)

class SimpleWrapper(AbstractSimulatorWrapper):
    def __init__(self, node, timestep):
        ########################## Load robot model and geometry
        robot = loadGo2()
        self.rmodel = robot.model
        self.geom_model = robot.collision_model
        self.visual_model = robot.visual_model

        # Ignore friction and kinematics limits
        """ for i in range(self.rmodel.nq):
            self.rmodel.lowerPositionLimit[i] = np.finfo("d").min
            self.rmodel.upperPositionLimit[i] = np.finfo("d").max 
        self.rmodel.lowerDryFrictionLimit[:] = 0
        self.rmodel.upperDryFrictionLimit[:] = 0"""

        # Load parameters from node
        self.params = {
            'max_fps': node.declare_parameter('max_fps', 30).value,
            'Kp': node.declare_parameter('Kp', 0.0).value,
            'Kd': node.declare_parameter('Kd', 0.0).value,
            'compliance': node.declare_parameter('compliance', 0.0).value,
            'material': node.declare_parameter('material', 'metal').value,
            'horizon': node.declare_parameter('horizon', 1000).value,
            'dt': node.declare_parameter('dt', 1e-3).value,
            'tol': node.declare_parameter('tol', 1e-6).value,
            'tol_rel': node.declare_parameter('tol_rel', 1e-6).value,
            'mu_prox': node.declare_parameter('mu_prox', 1e-4).value,
            'maxit': node.declare_parameter('maxit', 100).value,
            'warm_start': node.declare_parameter('warm_start', 1).value,
            'contact_solver': node.declare_parameter('contact_solver', 'PGS').value,
            'admm_update_rule': node.declare_parameter('admm_update_rule', 'spectral').value,
            'max_patch_size': node.declare_parameter('max_patch_size', 2).value,
            'patch_tolerance': node.declare_parameter('patch_tolerance', 1e-2).value,
        }

        self.init_simple(timestep)

    def init_simple(self, timestep):
        # Set simulation properties
        self.params["dt"] = timestep
        initial_q = self.rmodel.referenceConfigurations["standing"] #np.array([0, 0, 0.4, 0, 0, 0, 1, 0.0, 1.00, -2.51, 0.0, 1.09, -2.61, 0.2, 1.19, -2.59, -0.2, 1.32, -2.79])
        initial_q[2] += 0.2
        addFloor(self.geom_model, self.visual_model)
        setPhysicsProperties(self.geom_model, self.params["material"], self.params["compliance"])
        removeBVHModelsIfAny(self.geom_model)
        addSystemCollisionPairs(self.rmodel, self.geom_model, initial_q)

         # Remove all pair of collision which does not concern floor collision
        """ i = 0
        while i < len(self.geom_model.collisionPairs):
            cp = self.geom_model.collisionPairs[i]
            if self.geom_model.geometryObjects[cp.first].name != 'floor' and self.geom_model.geometryObjects[cp.second].name != 'floor':
                self.geom_model.removeCollisionPair(cp)
            else:
                i = i + 1 """

        # Create the simulator object
        self.simulator = SimpleSimulator(self.rmodel, self.geom_model, self.visual_model, initial_q, self.params)

    def step(self, tau_cmd):
        # Execute step and get new state
        torque_simu = np.zeros(self.rmodel.nv)
        torque_simu[6:] = tau_cmd
        q_current, v_current, a_current, f_current = self.simulator.execute(torque_simu)

        return q_current, v_current, a_current, f_current