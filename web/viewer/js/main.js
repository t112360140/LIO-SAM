// ==========================================
// 1. Three.js 初始化與場景設定
// ==========================================
const container = document.getElementById('canvas-container');
const scene = new THREE.Scene();

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 5000);
camera.position.set(0, 0, 100); 
camera.up.set(0, 0, 1); // Z-up

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x111111);
container.appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.05;

const gridHelper = new THREE.GridHelper(500, 50, 0x444444, 0x222222);
gridHelper.rotation.x = Math.PI / 2; 
scene.add(gridHelper);

// 動畫迴圈
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}
animate();

window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// UI 事件綁定
document.getElementById('grid-checkbox').addEventListener('change', (e) => {
    gridHelper.visible = e.target.checked;
});
document.getElementById('color-checkbox').addEventListener('change', () => {
    // 顏色切換時，清空快取並強制重新載入
    clearAllChunks();
    updateLODChunks();
});
document.getElementById('btn-refresh').addEventListener('click', () => {
    clearAllChunks();
    updateLODChunks();
});

// ==========================================
// 2. 地圖動態 LOD 調度系統 (升級版：視錐剔除 + 平滑過渡)
// ==========================================
let mapConfig = null;
// 記錄區塊狀態: Map<chunkId, { mesh: THREE.Points, stride: number, isTransitioning: boolean } | 'loading' | 'empty'>
const loadedChunks = new Map(); 
let updateTimer = null;

// 初始化：取得後端參數
async function initMapSystem() {
    try {
        const res = await fetch('/config');
        mapConfig = await res.json();
        
        document.getElementById('loading').style.display = 'none';
        
        if (mapConfig.isChunked) {
            document.getElementById('map-mode').innerText = `動態區塊 (${mapConfig.chunkSize}m)`;
            
            controls.addEventListener('change', () => {
                updateCameraUI();
                if (updateTimer) clearTimeout(updateTimer);
                // 縮短防抖時間，讓轉視角時反應更快
                updateTimer = setTimeout(updateLODChunks, 100); 
            });
            
            updateCameraUI();
            updateLODChunks();
        } else {
            document.getElementById('map-mode').innerText = "單一檔案 (Global)";
            loadedChunks.set('all', 'loading'); // 防護機制
            loadChunkData('all', 1);
        }
    } catch (e) {
        document.getElementById('loading').innerText = "無法連線至伺服器";
        document.getElementById('loading').style.color = "#ff3b3b";
    }
}

function updateCameraUI() {
    const p = camera.position;
    document.getElementById('camera-pos').innerText = 
        `X: ${p.x.toFixed(1)}, Y: ${p.y.toFixed(1)}, Z: ${p.z.toFixed(1)}`;
}

function updateChunkCountUI() {
    let count = 0;
    loadedChunks.forEach(val => {
        if (val !== 'loading' && val !== 'empty') count++;
    });
    document.getElementById('chunk-count').innerText = count;
}

// 核心大腦：推算需要哪些區塊 (加入視錐體 Frustum Culling)
function updateLODChunks() {
    if (!mapConfig || !mapConfig.isChunked) return;

    const camPos = camera.position;
    const cs = mapConfig.chunkSize;
    const useColor = document.getElementById('color-checkbox').checked;
    
    const cx = Math.floor(camPos.x / cs);
    const cy = Math.floor(camPos.y / cs);
    const cz = mapConfig.splitZ ? Math.floor(camPos.z / cs) : 0;

    // --- 建立相機的視錐體 (Frustum) ---
    const frustum = new THREE.Frustum();
    const projScreenMatrix = new THREE.Matrix4();
    camera.updateMatrixWorld();
    projScreenMatrix.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
    frustum.setFromProjectionMatrix(projScreenMatrix);

    const neededChunks = new Map(); 
    const searchRadius = 5; // 往外搜尋 5 格 (以 50m 來說是 250m)
    const zRadius = mapConfig.splitZ ? 1 : 0; 

    for (let dx = -searchRadius; dx <= searchRadius; dx++) {
        for (let dy = -searchRadius; dy <= searchRadius; dy++) {
            for (let dz = -zRadius; dz <= zRadius; dz++) {
                const nx = cx + dx;
                const ny = cy + dy;
                const nz = cz + dz;
                
                const chunkId = mapConfig.splitZ ? `${nx}_${ny}_${nz}` : `${nx}_${ny}`;
                
                // 1. 計算該區塊的 Bounding Box (邊界框)
                const min = new THREE.Vector3(nx * cs, ny * cs, mapConfig.splitZ ? nz * cs : -10000);
                const max = new THREE.Vector3((nx + 1) * cs, (ny + 1) * cs, mapConfig.splitZ ? (nz + 1) * cs : 10000);
                const box = new THREE.Box3(min, max);

                // 2. 防破洞機制：如果就在相機正周圍 (3x3)，強制保留，防止猛轉頭時看到空氣
                const isVeryClose = Math.max(Math.abs(dx), Math.abs(dy), Math.abs(dz)) <= 1;
                
                // 3. 視錐體剔除：不在視野內 且 不是極近距離 -> 跳過不載入
                if (!isVeryClose && !frustum.intersectsBox(box)) {
                    continue; 
                }

                // 4. 計算真實直線距離，決定精細度 (Stride)
                const center = new THREE.Vector3((nx + 0.5) * cs, (ny + 0.5) * cs, mapConfig.splitZ ? (nz + 0.5) * cs : 0);
                const dist = camPos.distanceTo(center);

                let stride = 1;
                if (dist > cs * 3.5) stride = 8;       // 超遠：低精度
                else if (dist > cs * 2.0) stride = 4;  // 中遠：中精度
                else if (dist > cs * 1.2) stride = 2;  // 近距：高精度

                neededChunks.set(chunkId, stride);
            }
        }
    }

    // --- 階段 1：清除視野外的區塊 ---
    for (const [loadedId, chunkData] of loadedChunks.entries()) {
        if (chunkData === 'loading' || chunkData === 'empty') {
            if (!neededChunks.has(loadedId)) loadedChunks.delete(loadedId);
            continue;
        }

        // 移出視野 -> 直接刪除釋放記憶體
        if (!neededChunks.has(loadedId)) {
            scene.remove(chunkData.mesh);
            chunkData.mesh.geometry.dispose();
            chunkData.mesh.material.dispose();
            loadedChunks.delete(loadedId);
        }
    }

    // --- 階段 2：平滑過渡載入 (Smooth Transition) ---
    for (const [neededId, targetStride] of neededChunks.entries()) {
        const currentData = loadedChunks.get(neededId);

        if (!currentData) {
            // 完全沒載入過的新區塊
            loadedChunks.set(neededId, 'loading');
            loadChunkData(neededId, targetStride, useColor);
        } else if (currentData !== 'loading' && currentData !== 'empty') {
            // 已存在，但需要「升級」或「降級」精細度
            if (currentData.stride !== targetStride && !currentData.isTransitioning) {
                // 標記為過渡中 (防止重複發送 fetch)
                currentData.isTransitioning = true; 
                // 將目前的 mesh 傳入，等新資料下載好再替換
                loadChunkData(neededId, targetStride, useColor, currentData.mesh);
            }
        }
    }
    updateChunkCountUI();
}

// 網路請求與點雲建構 (加入舊 Mesh 替換機制)
async function loadChunkData(chunkId, stride, useColor = true, oldMesh = null) {
    try {
        const response = await fetch(`/chunk/${chunkId}?stride=${stride}&color=${useColor}`);
        
        if (response.status === 404 || !response.ok) throw new Error("Chunk not found");

        const arrayBuffer = await response.arrayBuffer();
        if (arrayBuffer.byteLength === 0) throw new Error("Empty chunk");

        const interleavedFloat32 = new THREE.InterleavedBuffer(new Float32Array(arrayBuffer), 4);
        const interleavedUint8 = new THREE.InterleavedBuffer(new Uint8Array(arrayBuffer), 16);

        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.InterleavedBufferAttribute(interleavedFloat32, 3, 0));
        geometry.setAttribute('color', new THREE.InterleavedBufferAttribute(interleavedUint8, 3, 12, true));
        
        const material = new THREE.PointsMaterial({
            size: 0.3 * (stride * 0.5), 
            vertexColors: true,
            sizeAttenuation: true
        });
        
        const newPoints = new THREE.Points(geometry, material);

        // 如果在下載期間，相機轉開導致該區塊被拋棄，就不添加到場景
        if (!loadedChunks.has(chunkId)) {
            geometry.dispose();
            material.dispose();
            return;
        }

        // =====================================
        // 平滑過渡核心：先加新的，再刪舊的
        // =====================================
        scene.add(newPoints);

        if (oldMesh) {
            scene.remove(oldMesh);
            oldMesh.geometry.dispose();
            oldMesh.material.dispose();
        }

        // 更新狀態，解除過渡鎖定
        loadedChunks.set(chunkId, { mesh: newPoints, stride: stride, isTransitioning: false });
        updateChunkCountUI();

    } catch (error) {
        // 如果是替換失敗，保留舊的 mesh 並解除鎖定；否則標記為 empty
        if (oldMesh) {
            const currentData = loadedChunks.get(chunkId);
            if (currentData && currentData.mesh === oldMesh) {
                currentData.isTransitioning = false; 
            }
        } else {
            loadedChunks.set(chunkId, 'empty');
        }
    }
}

function clearAllChunks() {
    for (const [id, chunkData] of loadedChunks.entries()) {
        if (chunkData !== 'loading' && chunkData !== 'empty') {
            scene.remove(chunkData.mesh);
            chunkData.mesh.geometry.dispose();
            chunkData.mesh.material.dispose();
        }
    }
    loadedChunks.clear();
    updateChunkCountUI();
}

// 啟動系統
initMapSystem();
