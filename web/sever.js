// server.js — 华为云 IoTDA 数据代理服务（使用官方 SDK）
const http = require('http');

// ============ 使用华为云官方 SDK ============
const BasicCredentials = require('@huaweicloud/huaweicloud-sdk-core/auth/BasicCredentials').BasicCredentials;
const IoTDAClient = require('@huaweicloud/huaweicloud-sdk-iotda/v5/IoTDAClient').IoTDAClient;
const IoTDARegion = require('@huaweicloud/huaweicloud-sdk-iotda/v5/IoTDARegion').IoTDARegion;
const Region = require('@huaweicloud/huaweicloud-sdk-core/region/region').Region;
const ShowDeviceShadowRequest = require('@huaweicloud/huaweicloud-sdk-iotda/v5/model/ShowDeviceShadowRequest').ShowDeviceShadowRequest;

// ============ 配置 ============
const CONFIG = {
    AK: 'HPUAYXUM2NQRIJZPQI3Z',
    SK: 'F15FaOjORxTMNHsFJzfXqPu7bHu9Lcuvc9wnG6JY',
    projectId: 'c3685058dfed4d18acf85eaaeabf9e31',
    deviceId: '6a172279cbb0cf6bb95f1880_lab_inspection',

    // ⚠️ 关键配置：你的 IoTDA 实例类型
    //   'basic'  = 基础版（免费版），使用默认 endpoint，无需衍生认证
    //   'standard' = 标准版/企业版，需要自定义 endpoint + 衍生认证
    instanceType: 'standard',  // ← 根据你的实际情况修改！

    // 标准版/企业版专用：在华为云 IoTDA 控制台 → 总览 → 平台接入地址 → 应用侧 https 地址
    // 例如: 'xxxxxxxxxx.iotda.cn-north-4.myhuaweicloud.com'
    endpoint: '517c1b2cc8.st1.iotda-app.cn-north-4.myhuaweicloud.com',

    // 标准版/企业版专用：实例 ID，在 IoTDA 控制台 → 总览 页面查看
    instanceId: '08452361-00a2-4070-af74-518c89262e70',
};

// ============ 初始化 IoTDA 客户端 ============
function createIoTDAClient() {
    // 1. 创建 AK/SK 凭证
    const credentials = new BasicCredentials()
        .withAk(CONFIG.AK)
        .withSk(CONFIG.SK)
        .withProjectId(CONFIG.projectId);

    const builder = IoTDAClient.newBuilder()
        .withCredential(credentials);

    // 2. 根据实例类型配置 Region 和认证方式
    if (CONFIG.instanceType === 'standard' && CONFIG.endpoint) {
        // === 标准版/企业版 ===
        // 需要：① 自定义 endpoint  ② 衍生认证（Derived Predicate）
        // 自动补全 https:// 前缀
        const endpoint = CONFIG.endpoint.startsWith('http')
            ? CONFIG.endpoint
            : `https://${CONFIG.endpoint}`;
        console.log('🔧 使用标准版/企业版配置');
        console.log('   Endpoint:', endpoint);

        // 设置衍生认证判断函数
        credentials.withDerivedPredicate(
            (request) => BasicCredentials.getDefaultDerivedPredicate(request)
        );

        // 使用自定义 Region（传入完整 endpoint URL）
        builder.withRegion(new Region('cn-north-4', endpoint));
    } else {
        // === 基础版 ===
        // 使用 SDK 内置的 Region 配置
        console.log('🔧 使用基础版配置');
        builder.withRegion(IoTDARegion.CN_NORTH_4);
    }

    return builder.build();
}

const iotdaClient = createIoTDAClient();

// ============ 历史记录存储 ============
const MAX_HISTORY = 500; // 最多保留 500 条历史记录
const historyStore = [];

// 保存检测记录到历史（一批工位数据 = 一条批次记录）
function saveToHistory(records) {
    if (records.length === 0) return;

    const now = new Date();
    const timestamp = now.toISOString();
    const timeLabel = now.toLocaleString('zh-CN', { hour12: false });

    // 整个批次存为一条记录
    historyStore.push({
        timestamp,
        timeLabel,
        stations: records,  // 该批次包含的所有工位
    });

    // 超出上限时删除最旧的批次
    while (historyStore.length > MAX_HISTORY) {
        historyStore.shift();
    }

    console.log(`💾 已保存第 ${historyStore.length} 批次（${records.length} 个工位）`);
}

// ============ 查询设备影子 ============
async function queryDeviceShadow() {
    const request = new ShowDeviceShadowRequest();
    request.withDeviceId(CONFIG.deviceId);

    // 标准版/企业版必须传 Instance-Id
    if (CONFIG.instanceType === 'standard' && CONFIG.instanceId) {
        request.withInstanceId(CONFIG.instanceId);
    }

    const response = await iotdaClient.showDeviceShadow(request);
    // SDK 返回的响应对象直接包含 shadow、device_id 等字段
    return response;
}

// ============ 解析设备影子数据 → 前端需要的格式 ============
function parseShadowData(shadowResponse) {
    const shadowList = shadowResponse.shadow || [];
    if (shadowList.length === 0) return [];

    // 映射整数值 → 可读文本
    const chairMap = { 0: '归位', 1: '未归位' };
    const deviceMap = { 0: '正常', 1: '异常' };
    const messMap = { 0: '否', 1: '是' };
    const messDetailMap = { 0: '水杯', 1: '手机', 3: '无遗落物品' };

    // 遍历所有 shadow 条目（每个 service 对应一个工位或一类数据）
    const results = [];
    for (const entry of shadowList) {
        const reported = entry.reported?.properties || {};
        // 跳过没有有效数据的条目
        if (Object.keys(reported).length === 0) continue;

        results.push({
            service_id: entry.service_id || '--',
            station_id: reported.station_id !== undefined ? `S-${reported.station_id}` : '--',
            chair_status: chairMap[reported.chair_status] || '--',
            oscilloscope: deviceMap[reported.oscilloscope] || '--',
            siggen: deviceMap[reported.siggen] || '--',
            power_supply: deviceMap[reported.psu] || '--',
            multimeter: deviceMap[reported.dmm] || '--',
            has_mess: messMap[reported.has_mess] || '--',
            has_abnormality: (reported.oscilloscope === 1 || reported.siggen === 1 ||
                              reported.psu === 1 || reported.dmm === 1 ||
                              reported.chair_status === 1 || reported.has_mess === 1) ? '是' : '否',
            has_mass_detail: reported.mess_detail !== undefined && messDetailMap.hasOwnProperty(reported.mess_detail)
                ? messDetailMap[reported.mess_detail]
                : '',
        });
    }
    return results;
}

// ============ HTTP 服务器（供前端调用）============
const server = http.createServer(async (req, res) => {
    // CORS 允许前端跨域
    res.setHeader('Access-Control-Allow-Origin', '*');
    res.setHeader('Access-Control-Allow-Methods', 'GET, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'Content-Type');
    
    if (req.method === 'OPTIONS') {
        res.writeHead(204);
        res.end();
        return;
    }
    
    if (req.url === '/api/device-status' && req.method === 'GET') {
        try {
            const shadowData = await queryDeviceShadow();
            const parsed = parseShadowData(shadowData);
            // 保存到历史记录
            if (parsed.length > 0) {
                saveToHistory(parsed);
            }
            res.writeHead(200, { 'Content-Type': 'application/json; charset=utf-8' });
            res.end(JSON.stringify(parsed));
            console.log(`📡 成功获取设备影子数据，共 ${parsed.length} 个工位`);
        } catch (err) {
            const errMsg = err.data?.error_msg || err.message || String(err);
            const errCode = err.data?.error_code || '';
            const httpStatus = err.status || err.httpStatusCode || '';
            console.error(`❌ [${errCode}] HTTP ${httpStatus}: ${errMsg}`);

            // 针对常见错误给出中文诊断提示
            if (errCode === 'IOTDA.000021') {
                console.error('');
                console.error('🔍 错误诊断：403 Forbidden — 鉴权失败或无 IoTDA 访问权限');
                console.error('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
                console.error('请按以下步骤排查：');
                console.error('');
                console.error('1️⃣ 如果你是【标准版/企业版】IoTDA 实例：');
                console.error('   → 将上面 CONFIG.instanceType 改为 "standard"');
                console.error('   → 填写 CONFIG.endpoint（控制台→总览→平台接入地址→应用侧 https 地址）');
                console.error('   → 填写 CONFIG.instanceId（控制台→总览→实例 ID）');
                console.error('');
                console.error('2️⃣ 如果你是【基础版】且确认 AK/SK 正确：');
                console.error('   → 登录华为云 IAM 控制台');
                console.error('   → 找到 AK 对应的 IAM 用户');
                console.error('   → 授予 IoTDA 权限（如 IoTDA ReadOnlyAccess）');
                console.error('   → 确认已订阅开通 IoTDA 服务');
                console.error('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
            } else if (errCode === 'IOTDA.000001') {
                console.error('💡 设备不存在，请检查 deviceId 是否正确');
            } else if (httpStatus === 401) {
                console.error('💡 AK/SK 可能已过期或无效，请在华为云"我的凭证"中检查');
            }

            res.writeHead(500, { 'Content-Type': 'application/json; charset=utf-8' });
            res.end(JSON.stringify({ error: errMsg, code: errCode }));
        }
    } else if (req.url === '/api/device-history' && req.method === 'GET') {
        // 返回所有历史记录（最新批次在前，展开为逐工位行）
        const flat = [];
        for (let i = historyStore.length - 1; i >= 0; i--) {
            const batch = historyStore[i];
            for (const station of batch.stations) {
                flat.push({
                    ...station,
                    timestamp: batch.timestamp,
                    timeLabel: batch.timeLabel,
                    batchIndex: i + 1,
                });
            }
        }
        res.writeHead(200, { 'Content-Type': 'application/json; charset=utf-8' });
        res.end(JSON.stringify(flat));
    } else if (req.url === '/api/device-history/clear' && req.method === 'POST') {
        // 清空历史记录
        const count = historyStore.length;
        historyStore.length = 0;
        res.writeHead(200, { 'Content-Type': 'application/json; charset=utf-8' });
        res.end(JSON.stringify({ cleared: count }));
        console.log(`🗑 已清空 ${count} 条历史记录`);
    } else {
        res.writeHead(404);
        res.end('Not Found');
    }
});

server.on('error', (err) => {
    if (err.code === 'EADDRINUSE') {
        console.error('❌ 端口 3000 已被占用！请先关闭占用该端口的进程：');
        console.error('   netstat -ano | findstr :3000');
        console.error('   taskkill /PID <PID> /F');
    } else {
        console.error('❌ 服务器错误:', err.message);
    }
    process.exit(1);
});

server.listen(3000, () => {
    console.log('✅ 代理服务已启动: http://localhost:3000');
    console.log('📡 API 地址: http://localhost:3000/api/device-status');
});