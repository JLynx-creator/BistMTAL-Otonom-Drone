let scene, camera, renderer, controls;
let drones = { safe: null, fast: null };
let paths = { safe: null, fast: null };

function init3D() {
    const container = document.getElementById('three-container');
    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);

    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);


    const gridHelper = new THREE.GridHelper(50, 10, 0x444444, 0x222222);
    gridHelper.position.set(25, 0, 25);
    scene.add(gridHelper);


    const light = new THREE.PointLight(0xffffff, 1, 100);
    light.position.set(25, 50, 25);
    scene.add(light);
    scene.add(new THREE.AmbientLight(0x404040));

    camera.position.set(70, 70, 70);
    controls.update();

    animate3D();
}

function animate3D() {
    requestAnimationFrame(animate3D);
    controls.update();
    renderer.render(scene, camera);
}


function update3DScene(data) {

    while (scene.children.length > 0) { scene.remove(scene.children[0]); }
    init3D();


    data.obstacles.forEach(obs => {
        const geo = new THREE.SphereGeometry(obs.r, 32, 32);
        const mat = new THREE.MeshPhongMaterial({
            color: 0xffffff, transparent: true, opacity: 0.1, wireframe: true
        });
        const sphere = new THREE.Mesh(geo, mat);
        sphere.position.set(obs.x, obs.z, obs.y);
        scene.add(sphere);
    });


    drawRoute(data.safe_path, 0x00ffff, "GÜVENLİ");
    drawRoute(data.fast_path, 0xff0000, "HIZLI");
}

function drawRoute(points, color, label) {
    const curvePoints = points.map(p => new THREE.Vector3(p[0], p[2], p[1]));
    const geometry = new THREE.BufferGeometry().setFromPoints(curvePoints);
    const material = new THREE.LineBasicMaterial({ color: color, linewidth: 2 });
    const line = new THREE.Line(geometry, material);
    scene.add(line);


    const droneGeo = new THREE.BoxGeometry(1, 1, 1);
    const droneMat = new THREE.MeshLambertMaterial({ color: color });
    const drone = new THREE.Mesh(droneGeo, droneMat);
    scene.add(drone);


    let step = 0;
    function move() {
        if (step < curvePoints.length) {
            drone.position.copy(curvePoints[step]);
            step++;
            setTimeout(move, 50);
        }
    }
    move();
}


window.onload = init3D;