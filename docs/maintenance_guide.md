# AstraDroneOpen 维护者操作手册

本手册面向项目维护者，说明日常维护流程和常用操作。

## 1. 日常维护流程

### 1.1 处理 Issue

1. 查看新 Issue，判断问题类型（bug、功能请求、文档问题等）
2. 确认问题是否在允许修改范围内
3. 如果是高风险问题（涉及核心逻辑），记录并通知负责人
4. 如果是低风险问题，创建分支并修复

### 1.2 处理 PR

1. 检查 PR 是否填写了模板
2. 检查 GitHub Actions 是否通过
3. 检查修改内容是否在允许范围内
4. 检查是否涉及核心逻辑
5. 检查 AI 使用说明
6. 审核通过后合并，不通过则说明原因

### 1.3 定期检查

- 每周检查一次 Issue 和 PR 状态
- 每周运行一次代码扫描，记录新发现的问题
- 每月更新一次维护报告

## 2. 常用 Git 命令

### 2.1 基本操作

```bash
# 查看当前状态
git status

# 查看分支
git branch -a

# 切换分支
git checkout branch-name

# 创建并切换新分支
git checkout -b new-branch-name

# 拉取最新代码
git pull origin main

# 查看提交历史
git log --oneline -10
```

### 2.2 分支管理

```bash
# 创建新分支
git checkout -b intern/maintenance-docs

# 推送分支到远程
git push origin intern/maintenance-docs

# 删除本地分支
git branch -d branch-name

# 删除远程分支
git push origin --delete branch-name
```

### 2.3 提交修改

```bash
# 添加文件
git add filename

# 添加所有修改
git add .

# 提交
git commit -m "描述修改内容"

# 推送
git push origin branch-name
```

### 2.4 同步远程

```bash
# 获取远程最新代码
git fetch origin

# 重置到远程 main
git reset --hard origin/main

# 清理未跟踪文件
git clean -fd
```

## 3. 常用检查命令

### 3.1 Python 语法检查

```bash
# 检查单个文件
python3 -m py_compile filename.py

# 检查所有 Python 文件
find . -name "*.py" -print0 | xargs -0 -n1 python3 -m py_compile
```

### 3.2 XML/launch 文件检查

```bash
# 检查单个文件
xmllint --noout filename.xml

# 检查所有 launch 文件
find . -name "*.launch" -print0 | xargs -0 -n1 xmllint --noout

# 检查所有 package.xml
find . -name "package.xml" -print0 | xargs -0 -n1 xmllint --noout
```

### 3.3 Shell 脚本检查

```bash
# 检查单个脚本
bash -n script.sh

# 检查所有 shell 脚本
find . -name "*.sh" -print0 | xargs -0 -n1 bash -n
```

### 3.4 查找残留内容

```bash
# 查找 TODO/FIXME
grep -RIn "TODO\|FIXME" . --exclude-dir=.git --exclude-dir=build

# 查找 AI 残留
grep -RIn "contentReference\|Lorem ipsum\|placeholder" . --exclude-dir=.git

# 查找脚本名不一致
grep -RIn "pc_installer\|build_AstraDrone\|build_sim_workspace" README.md docs scripts
```

## 4. PR 提交流程

1. 确保在最新的 main 分支上：
   ```bash
   git checkout main
   git pull origin main
   ```

2. 创建新分支：
   ```bash
   git checkout -b your-name/fix-description
   ```

3. 进行修改

4. 运行本地检查（见第 3 节）

5. 提交修改：
   ```bash
   git add .
   git commit -m "描述修改内容"
   ```

6. 推送分支：
   ```bash
   git push origin your-name/fix-description
   ```

7. 在 GitHub 上创建 PR，填写模板

8. 等待 GitHub Actions 运行

9. 如果 Actions 失败，修复后重新推送

10. 等待负责人审核

## 5. GitHub Actions 失败处理

### 5.1 查看失败原因

1. 在 PR 页面点击「Actions」标签
2. 点击失败的 workflow
3. 查看具体的失败步骤和错误信息

### 5.2 常见失败原因

- **Python 语法错误**：检查 Python 文件语法
- **XML 格式错误**：检查 XML/launch 文件格式
- **Shell 语法错误**：检查 shell 脚本语法
- **文件编码问题**：确保文件使用 UTF-8 编码

### 5.3 修复流程

1. 根据错误信息定位问题文件
2. 在本地修复问题
3. 重新运行本地检查确认修复
4. 提交并推送修复
5. 等待 Actions 重新运行

## 6. 高风险目录说明

以下目录属于核心逻辑，**禁止擅自修改**：

| 目录 | 内容 | 风险等级 |
|------|------|----------|
| `Control/` | 核心控制逻辑 | 高 |
| `Planner/` | 路径规划逻辑 | 高 |
| `SLAM/` | 算法逻辑 | 高 |
| `MissionControl/` | 任务控制逻辑 | 高 |
| PX4/MAVROS 配置 | 飞控参数 | 高 |
| 第三方依赖 | 许可证、版权 | 高 |

如发现上述目录存在问题，只记录到 Issue 或报告中，不直接修改。

## 7. AI 使用规范

### 7.1 允许使用 AI 的场景

- 理解代码、解释目录结构、分析报错日志
- 生成文档初稿、PR 描述草稿、Issue 草稿
- 生成 GitHub Actions 初稿
- 辅助查找低风险问题
- 辅助整理检查命令和报告格式

### 7.2 禁止或需确认的场景

- 让 AI 大规模重构项目
- 未理解就提交 AI 生成代码
- 让 AI 修改飞控、规划、控制、SLAM 核心逻辑
- 让 AI 编造测试结果或项目能力
- 让 AI 作为 commit author 或直接提交到 main

### 7.3 AI 使用底线

AI 可以辅助，但不能替代验证。凡是 AI 生成的代码、文档或配置，提交前必须由本人阅读、理解、检查，并在 PR 中说明使用情况。

## 8. 注意事项

- 所有修改必须通过 PR 提交，禁止直接修改 main
- PR 必须填写模板，不允许只写"已完成"
- 不确定的问题先记录，不要擅自处理
- 不得编造验证结果，不得夸大项目成熟度
- 安全问题不要公开发布，通过私下渠道反馈
