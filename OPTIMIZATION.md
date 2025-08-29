# GPS Globe Visualization Optimization

## 问题解决

### 原始问题
- `earth_globe` topic 持续发布，每2秒一次
- Marker绘制比较费时，影响RViz性能
- 不必要的重复渲染消耗资源

### 优化方案

#### 1. 发布一次模式 (推荐)
**默认行为**: `publish_once=true`
- 只发布一次marker
- 设置marker lifetime为0（永久存在）
- 发布后自动停止timer
- **大幅减少渲染负载**

#### 2. 可配置发布频率
**持续模式**: `publish_once=false` 
- 可自定义发布频率（Hz）
- 适用于需要动态更新的场景
- 默认0.5Hz（比原来的0.5Hz更低）

## 性能对比

| 模式 | 发布频率 | 渲染次数/分钟 | 性能影响 |
|------|----------|---------------|----------|
| 原始 | 每2秒 | 30次 | 高 |
| 优化(默认) | 仅1次 | 1次 | 极低 |
| 优化(0.5Hz) | 每2秒 | 30次 | 低 |
| 优化(0.1Hz) | 每10秒 | 6次 | 极低 |

## 使用方法

### 默认优化模式（推荐）
```bash
# 只发布一次，marker永久显示
ros2 launch geodetic_points globe_viz.launch.py
```

### 低频更新模式
```bash
# 每10秒更新一次
ros2 launch geodetic_points globe_viz.launch.py \
  publish_once:=false \
  publish_frequency:=0.1
```

### 关键优化特性
1. **Marker Lifetime**: 设置为0（永久存在）
2. **Timer控制**: publish_once模式下自动停止
3. **参数化配置**: 可通过launch参数灵活控制
4. **详细日志**: 显示优化状态和参数

## 日志输出示例

### 优化模式（只发布一次）
```
[INFO] publish_once: True
[INFO] timer_period: 1.00 seconds
[INFO] === Published Globe Marker ===
[INFO] Lifetime: 0s (0=forever)
[INFO] publish_once enabled - stopping timer
```

### 持续模式（可配置频率）
```
[INFO] publish_once: False
[INFO] publish_frequency: 0.1 Hz
[INFO] timer_period: 10.00 seconds
[INFO] === Published Globe Marker ===
[INFO] Lifetime: 0s (0=forever)
```

## 总结
通过这些优化，地球可视化的渲染性能得到显著提升，特别是在默认的"发布一次"模式下，几乎完全消除了不必要的重复渲染开销。