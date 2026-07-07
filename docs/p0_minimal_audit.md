# P0 最小审计清单

审计日期：2026-07-07
审计范围：AstraDroneOpen 项目基础检查

## 1. 已修复问题

### 1.1 AI 残留清理 ✅

| 文件 | 问题 | 状态 |
|------|------|------|
| docs/00-AstraDrone开发教程.md:149 | contentReference[oaicite:0] | 已修复 |
| docs/00-AstraDrone开发教程.md:166 | contentReference[oaicite:1] | 已修复 |
| docs/00-AstraDrone开发教程.md:254 | contentReference[oaicite:2] | 已修复 |
| docs/00-AstraDrone开发教程.md:255 | contentReference[oaicite:3] | 已修复 |

### 1.2 脚本名修正 ✅

| 文件 | 旧引用 | 新引用 | 数量 |
|------|--------|--------|------|
| README.md | build_AstraDrone_ros1.bin | build_AstraDrone_ros1_x86.bin | ~8处 |
| README.md | build_sim_workspace.bin | build_sim_workspace_x86.bin | ~4处 |
| README.md | pc_installer.bin | pc_installer_x86.bin | ~4处 |
| README.md | onboard_installer.bin | onboard_installer_x86.bin | ~2处 |

### 1.3 维护文件添加 ✅

| 文件 | 说明 |
|------|------|
| CONTRIBUTING.md | 贡献指南 |
| SECURITY.md | 安全政策 |
| .github/PULL_REQUEST_TEMPLATE.md | PR 模板 |
| docs/maintenance_guide.md | 维护者操作手册 |

### 1.4 CI 添加 ✅

| 文件 | 说明 |
|------|------|
| .github/workflows/static-check.yml | 基础静态检查 |

## 2. 未确认问题（记录但不修改）

### 2.1 package.xml 模板残留

- **package.xml 总数**：226 个（需以最新命令结果为准）
- **疑似模板残留数量**：待复核；PR #2 曾记录 38 个疑似模板残留，需要重新运行命令确认
- **问题**：部分 package.xml 可能存在模板残留内容
- **风险**：数量多，批量修改可能影响 ROS/catkin 编译和第三方包声明
- **建议**：暂不批量修改；后续由负责人确认清理范围、白名单和验证方式

### 2.2 Python 2 语法

- **文件**：
  - AstraDrone_ros1_ws/src/Planner/ego-planner/Utils/quaternion_ge_utils.py
  - AstraDrone_ros1_ws/src/Planner/ego-planner/Utils/rotation_ge_utils.py
  - AstraDrone_ros1_ws/src/Planner/ego-planner/Utils/graph_search_utils.py
- **问题**：使用 Python 2 语法（print 语句等）
- **风险**：可能故意保持 Python 2 兼容
- **建议**：需要确认是否需要迁移到 Python 3，还是保持上游兼容

### 2.3 README 待补充标记

- **位置**：
  - README.md:38 - ROS2 模块说明（待补充）
  - README.md:158 - 机载电脑快速启动（还未完善）
  - README.md:188 - 启动程序（还未完善）
- **问题**：内容确实未完成
- **建议**：不能删除标记，需要后续补充内容

### 2.4 catkin_simple 递归测试目录

- **位置**：AstraDrone_ros1_ws/src/Utils/catkin_simple/test/
- **问题**：测试目录存在递归结构
- **风险**：第三方代码，不建议修改
- **建议**：保持原样

## 3. 高风险问题（只记录不修改）

### 3.1 核心逻辑目录

以下目录属于核心逻辑，禁止擅自修改：

| 目录 | 内容 | 风险等级 |
|------|------|----------|
| Control/ | 核心控制逻辑 | 高 |
| Planner/ | 路径规划逻辑 | 高 |
| SLAM/ | 算法逻辑 | 高 |
| MissionControl/ | 任务控制逻辑 | 高 |
| PX4/MAVROS 配置 | 飞控参数 | 高 |

### 3.2 第三方依赖

以下第三方依赖不建议修改：

- Livox-SDK / Livox-SDK2
- apriltag
- GeographicLib
- librealsense
- nlopt

## 4. 统计信息

| 项目 | 数量 |
|------|------|
| package.xml 文件总数 | 226 |
| 疑似模板残留数量 | 待复核（曾记录 38 个） |
| 已修复 AI 残留 | 4 处 |
| 已修正脚本名引用 | ~20 处 |
| 待补充文档标记 | 3 处 |
| 核心逻辑目录 | 5 个 |
| 第三方依赖 | 5+ 个 |

## 5. 需要负责人确认的问题

1. package.xml 模板残留是否需要批量清理；如果清理，是否先建立白名单和单独 PR。
2. ego-planner Utils 中疑似 Python 2 语法文件是否需要迁移到 Python 3，还是保持上游兼容。
3. CI 是否需要排除 third_party、build、devel、install、log 等目录。
4. 是否启用安全邮箱或 GitHub Security Advisory 作为 SECURITY.md 的正式反馈渠道。
5. 是否建立 branch protection，禁止直接 push main，并要求 PR + Actions 通过后才能合并。

## 6. 本次运行/建议运行的检查命令

```bash
# Python 语法检查
find . -name "*.py" -print0 | xargs -0 -n1 python3 -m py_compile

# 查 TODO / FIXME / 残留内容
grep -RIn "TODO\|FIXME\|todo.todo\|待补充\|还未完善\|contentReference" . \
  --exclude-dir=.git \
  --exclude-dir=build \
  --exclude-dir=devel \
  --exclude-dir=install \
  --exclude-dir=log

# 查脚本名不一致
grep -RIn "pc_installer\|build_AstraDrone\|build_sim_workspace" README.md docs scripts . \
  --exclude-dir=.git

# 查 package.xml 总数
find . -name "package.xml" | wc -l
```

## 7. 后续建议

1. **短期**：合并 PR #3；整改并合并 PR #1；最后更新并合并 PR #4
2. **中期**：逐个检查 package.xml，确认是否需要清理模板残留
3. **长期**：补充 README 中标记为"待补充"的内容

---

*审计更新时间：2026-07-07*
