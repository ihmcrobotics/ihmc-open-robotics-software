// Use the same host as the page, but fixed port 8111
const wsProtocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
const wsHost = location.hostname; // only the hostname, not port
const wsPort = 8765;
const ws = new WebSocket(`${wsProtocol}//${wsHost}:${wsPort}`);

ws.onopen = () => log("Connected to server");
ws.onclose = () => log("Disconnected from server");
ws.onmessage = (event) => {
    log("Received: " + event.data);

    try {
        const data = JSON.parse(event.data);

        // Update robot position if message contains it
        if (data.x !== undefined && data.y !== undefined) {
            robot.position.set(data.x, 0.5, data.y);
        }
        if (data.theta !== undefined) {
            robot.rotation.y = data.theta;
        }
    } catch (e) {
        // Non-JSON, ignore
    }
};

function log(message) {
    const logArea = document.getElementById("log");
    logArea.value += message + "\n";
    logArea.scrollTop = logArea.scrollHeight;
}

function getDirection() {
    return document.querySelector('input[name="direction"]:checked').value;
}

function buildCommand(enable) {
    return {
        enable_continuous_hiking: enable,
        steps_before_safety_stop: parseInt(document.getElementById("steps_before_safety_stop").value) || 0,
        walk_forwards: getDirection() === "forward",
        walk_backwards: getDirection() === "backward",
        side_step: document.getElementById("side_step").checked,
        left_direction: document.getElementById("left_direction").checked,
        square_up_to_goal: document.getElementById("square_up_to_goal").checked,
        use_astar_footstep_planner: document.getElementById("use_astar_footstep_planner").checked,
        use_monte_carlo_footstep_planner: document.getElementById("use_monte_carlo_footstep_planner").checked,
        use_previous_plan_as_reference: document.getElementById("use_previous_plan_as_reference").checked,
        use_monte_carlo_plan_as_reference: document.getElementById("use_monte_carlo_plan_as_reference").checked,
        use_joystick_controller: document.getElementById("use_joystick_controller").checked,
        forward_value: parseFloat(document.getElementById("forward_value").value) || 0.0,
        lateral_value: parseFloat(document.getElementById("lateral_value").value) || 0.0,
        turning_value: parseFloat(document.getElementById("turning_value").value) || 0.0
    };
}

// Hook up Start/Stop buttons
document.getElementById("startBtn").onclick = () => {
    const cmd = buildCommand(true);
    const json = JSON.stringify(cmd);
    ws.send(json);
    log("Sent: " + json);
};

document.getElementById("stopBtn").onclick = () => {
    const cmd = buildCommand(false);
    const json = JSON.stringify(cmd);
    ws.send(json);
    log("Sent: " + json);
};

function stopServer() {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send("STOP_SERVER");
        log("Sent: STOP_SERVER");
    }
}

// === 3D Scene Setup ===
let scene, camera, renderer, robot, controls;

function init3D() {
    const container = document.getElementById("world");

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xeeeeee);

    camera = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.1, 1000);
    camera.position.set(3, 3, 5);

    renderer = new THREE.WebGLRenderer({antialias: true});
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);

    // Lights
    const light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(5, 10, 7).normalize();
    scene.add(light);

    // Ground plane
    const planeGeometry = new THREE.PlaneGeometry(20, 20);
    const planeMaterial = new THREE.MeshPhongMaterial({color: 0x999999, depthWrite: true});
    const plane = new THREE.Mesh(planeGeometry, planeMaterial);
    plane.rotation.x = -Math.PI / 2;
    scene.add(plane);

    // Simple "robot" box
    const robotGeometry = new THREE.BoxGeometry(0.5, 1.0, 0.5);
    const robotMaterial = new THREE.MeshPhongMaterial({color: 0x0077ff});
    robot = new THREE.Mesh(robotGeometry, robotMaterial);
    robot.position.set(0, 0.5, 0);
    scene.add(robot);

    controls = new THREE.OrbitControls(camera, renderer.domElement);

    animate();
}

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

init3D();
