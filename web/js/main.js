// ==========================================
// 1. UI 狀態與定時器邏輯
// ==========================================
let autoUpdatePath = true;
let pathInterval;

let currentLoopLines = null;
let showLoopEdges = true;
let loopInterval;

// 定期抓取狀態
let processing = false;
setInterval(() => {
    fetch('/status').then(r => r.json()).then(data => {
        document.getElementById('bar').style.width = data.progress + '%';
        document.getElementById('bar').innerText = data.progress.toFixed(1) + '%';
        document.getElementById('msg').innerText = data.current + ' / ' + data.total;

        const usedGB = (data.ram_used / 1073741824).toFixed(1);
        const totalGB = (data.ram_total / 1073741824).toFixed(1);

        document.getElementById('cpu-text').innerText = data.cpu.toFixed(1) + '%';
        document.getElementById('ram-text').innerText = `${usedGB} / ${totalGB} GB (${data.ram.toFixed(1)}%)`;
        document.getElementById('cpu-bar').style.width = data.cpu + '%';
        document.getElementById('ram-bar').style.width = data.ram + '%';
        document.getElementById('cpu-bar').style.backgroundColor = (data.cpu > 85) ? '#f44336' : '#2196F3';
        document.getElementById('ram-bar').style.backgroundColor = (data.ram > 85) ? '#f44336' : '#9C27B0';
        
        const level = data.status.match(/^(\[INFO\]|\[ERROR\])/);
        if (level !== null) {
            document.getElementById('status').style.color = (level[1] === '[INFO]') ? "#3bff45" : ((level[1] === '[ERROR]') ? "#ff3b3b" : "#ffeb3b");
            document.getElementById('status').innerText = data.status.slice(level[1].length+1);
        } else {
            document.getElementById('status').style.color = "#ffeb3b";
            document.getElementById('status').innerText = data.status;
        }

        if (data.processing && !processing) {
            if (!autoUpdatePath) toggleAutoPath();
            clearTrajectoryPath();
            clearPointCloudPreview();
            clearLoopEdges();
        }
        if (!data.processing && processing) {
            if (autoUpdatePath) toggleAutoPath();
            updateTrajectoryPath();
            loadPointCloudPreview();
        }
        processing = data.processing
    }).catch(e => console.log("等待伺服器連線..."));
}, 1000); 

async function triggerSave() {
    const req = await fetch('/save');
    if(!req.ok) {
        alert('存檔指令傳送失敗!');
        return;
    }
    const status = await req.json();
    if (status.status === 'saving_queued') alert('存檔指令已加入佇列！');
    else if (status.status === 'saved_directly')alert('存檔完成!');
}

function toggleAutoPath() {
    autoUpdatePath = !autoUpdatePath;
    const btn = document.getElementById('btn-toggle-path');
    if (autoUpdatePath) {
        btn.innerText = "⏸️ 暫停自動更新軌跡";
        btn.style.background = "#008cba";
        startPathUpdate();
    } else {
        btn.innerText = "▶️ 恢復自動更新軌跡";
        btn.style.background = "#555";
        clearInterval(pathInterval);
    }
}

// ==========================================
// 2. Three.js 初始化與場景設定
// ==========================================
const container = document.getElementById('canvas-container');
const scene = new THREE.Scene();
// 添加一點霧效，讓遠處的點雲稍微融入背景
// scene.fog = new THREE.FogExp2(0x111111, 0.002);

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 5000);
camera.position.set(0, 0, 100); // 預設視角高度
camera.up.set(0, 0, 1); // Z-up 座標系 (符合 ROS/LIO-SAM 習慣)

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x111111);
container.appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.05;

// 添加坐標軸輔助 (RGB = XYZ)
// const axesHelper = new THREE.AxesHelper(10);
// scene.add(axesHelper);

// Grid 網格輔助 (設定在 XY 平面)
const gridHelper = new THREE.GridHelper(200, 20, 0x444444, 0x222222);
gridHelper.rotation.x = Math.PI / 2; // 將網格翻轉到 Z-up 的 XY 平面上
scene.add(gridHelper);

// 全域物件參考
let currentPathLine = null;
let currentPointCloud = null;

// 動畫迴圈
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}
animate();

// RWD 視窗大小調整
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// ==========================================
// 3. 資料拉取與渲染邏輯 (Binary Zero-copy)
// ==========================================

// 定期拉取並更新軌跡
async function updateTrajectoryPath() {
    try {
        const response = await fetch('/path');
        if (!response.ok) return;
        
        const arrayBuffer = await response.arrayBuffer();
        const floatArray = new Float32Array(arrayBuffer);
        
        // 只有當軌跡真的有更新時才重繪 (透過長度判斷)
        if (currentPathLine && currentPathLine.geometry.attributes.position.count * 3 === floatArray.length) {
            return;
        }

        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(floatArray, 3));
        
        const material = new THREE.LineBasicMaterial({ color: 0xffeb3b, linewidth: 2 });
        
        if (currentPathLine) {
            scene.remove(currentPathLine);
            currentPathLine.geometry.dispose();
            currentPathLine.material.dispose();
        }
        
        currentPathLine = new THREE.Line(geometry, material);
        scene.add(currentPathLine);

        if (showLoopEdges) updateLoopEdges();
        
        // 讓相機自動追蹤最新的點 (可選功能)
        /*
        if (floatArray.length >= 3) {
            const lastX = floatArray[floatArray.length - 3];
            const lastY = floatArray[floatArray.length - 2];
            const lastZ = floatArray[floatArray.length - 1];
            controls.target.set(lastX, lastY, lastZ);
        }
        */
    } catch (e) {
        // Ignore fetch errors while server might be busy
    }
}
function clearTrajectoryPath() {
    if (currentPathLine) {
        scene.remove(currentPathLine);
        currentPathLine.geometry.dispose();
        currentPathLine.material.dispose();
    }
}

function startPathUpdate() {
    pathInterval = setInterval(updateTrajectoryPath, 2000);
}
startPathUpdate();

async function updateLoopEdges() {
    if (!showLoopEdges) return;
    try {
        const response = await fetch('/loop_indices');
        if (!response.ok) return;
        
        const arrayBuffer = await response.arrayBuffer();
        const indicesArray = new Uint32Array(arrayBuffer);
        
        if (!currentPathLine) return;
        
        const pathPositions = currentPathLine.geometry.attributes.position.array;
        
        const vertices = [];
        for (let i = 0; i < indicesArray.length; i += 2) {
            const idx1 = indicesArray[i];
            const idx2 = indicesArray[i+1];

            if (idx1 * 3 + 2 < pathPositions.length && idx2 * 3 + 2 < pathPositions.length) {
                vertices.push(
                    pathPositions[idx1 * 3], pathPositions[idx1 * 3 + 1], pathPositions[idx1 * 3 + 2],
                    pathPositions[idx2 * 3], pathPositions[idx2 * 3 + 1], pathPositions[idx2 * 3 + 2]
                );
            }
        }

        const floatVertices = new Float32Array(vertices);

        if (currentLoopLines && currentLoopLines.geometry.attributes.position.count * 3 === floatVertices.length) {
            return;
        }

        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(floatVertices, 3));
        const material = new THREE.LineBasicMaterial({ color: 0x00bfff, linewidth: 2 });
        
        if (currentLoopLines) {
            scene.remove(currentLoopLines);
            currentLoopLines.geometry.dispose();
            currentLoopLines.material.dispose();
        }
        
        currentLoopLines = new THREE.LineSegments(geometry, material);
        scene.add(currentLoopLines);
        
    } catch (e) {
        // Ignore fetch errors
    }
}

function clearLoopEdges() {
    if (currentLoopLines) {
        scene.remove(currentLoopLines);
        currentLoopLines.geometry.dispose();
        currentLoopLines.material.dispose();
        currentLoopLines = null;
    }
}

function toggleLoopEdges() {
    showLoopEdges = document.getElementById('loop-checkbox').checked;
    if (showLoopEdges) {
        updateLoopEdges();
        loopInterval = setInterval(updateLoopEdges, 2000);
    } else {
        clearInterval(loopInterval);
        clearLoopEdges();
    }
}
toggleLoopEdges();

// 手動拉取全局點雲
async function loadPointCloudPreview() {
    const loading = document.getElementById('loading');
    loading.style.display = 'block';

    const useColor = document.getElementById('color-checkbox').checked;
    
    try {
        const response = await fetch(`/map_preview?color=${useColor}`);
        if (!response.ok) throw new Error("伺服器回應錯誤");

        const arrayBuffer = await response.arrayBuffer();
        const floatArray = new Float32Array(arrayBuffer);
        
        // 建立交錯緩衝 (x, y, z, r, g, b)
        const interleavedFloat32 = new THREE.InterleavedBuffer(new Float32Array(arrayBuffer), 4);  // 16 bytes = 4 個 float
        const interleavedUint8 = new THREE.InterleavedBuffer(new Uint8Array(arrayBuffer), 16);     // 16 bytes = 16 個 uint8

        const geometry = new THREE.BufferGeometry();

        // xyz 從偏移 0 開始讀 3 個 float
        geometry.setAttribute('position', new THREE.InterleavedBufferAttribute(interleavedFloat32, 3, 0));

        // rgb 從偏移 12 開始讀 3 個 uint8 (第四個參數 true 會告訴 WebGL 自動將 0-255 map 到 0.0-1.0)
        geometry.setAttribute('color', new THREE.InterleavedBufferAttribute(interleavedUint8, 3, 12, true));
        
        const material = new THREE.PointsMaterial({
            size: 0.5, // 點的大小
            vertexColors: true,
            sizeAttenuation: true
        });
        
        if (currentPointCloud) {
            scene.remove(currentPointCloud);
            currentPointCloud.geometry.dispose();
            currentPointCloud.material.dispose();
        }
        
        currentPointCloud = new THREE.Points(geometry, material);
        scene.add(currentPointCloud);
        
        console.log(`成功載入全局點雲！總共 ${floatArray.length / 6} 個點。`);
    } catch (error) {
        console.error("載入點雲失敗:", error);
        alert("載入點雲失敗，請確保 LIO-SAM 正在運行中。");
    } finally {
        loading.style.display = 'none';
    }
}
function clearPointCloudPreview() {
    if (currentPointCloud) {
        scene.remove(currentPointCloud);
        currentPointCloud.geometry.dispose();
        currentPointCloud.material.dispose();
    }
}