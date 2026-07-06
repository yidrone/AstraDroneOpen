# 贡献指南

感谢你对 AstraDroneOpen 项目的关注！在参与贡献之前，请仔细阅读以下规则。

## 重要原则

1. **禁止直接修改 main 分支** — 所有修改必须通过 Issue → Branch → PR → 审核流程
2. **禁止擅自修改核心逻辑** — Control、Planner、SLAM、MissionControl 目录的代码逻辑
3. **禁止擅自修改飞控配置** — PX4、MAVROS 相关的飞控参数和配置
4. **AI 可以辅助，但不能替代验证** — 所有 AI 生成的内容必须由本人检查后才能提交

## 如何提交 Issue

1. 在 GitHub 上点击「New Issue」
2. 写清楚问题描述、复现步骤、期望行为
3. 如果是安全或飞行相关问题，请不要公开发布，通过私下渠道反馈

## 如何提交代码修改

### 第一步：新建分支

```bash
# 确保在最新的 main 分支上
git checkout main
git pull origin main

# 新建分支（建议命名：你的名字/修改内容）
git checkout -b your-name/fix-description
```

### 第二步：修改代码

- 只做小范围、低风险修改
- 发现高风险问题只记录，不直接修改
- 使用 AI 辅助时，必须自己理解、检查后再提交

### 第三步：本地检查

```bash
# Python 语法检查
find . -name "*.py" -print0 | xargs -0 -n1 python3 -m py_compile

# XML/launch 文件检查
find . -name "*.launch" -print0 | xargs -0 -n1 xmllint --noout
find . -name "package.xml" -print0 | xargs -0 -n1 xmllint --noout

# Shell 脚本检查
find . -name "*.sh" -print0 | xargs -0 -n1 bash -n
```

### 第四步：提交 PR

1. 推送分支到远程：`git push origin your-name/fix-description`
2. 在 GitHub 上创建 Pull Request
3. **必须填写 PR 模板**，写清楚：
   - 改了什么
   - 为什么改
   - 怎么验证的
   - 是否使用 AI
   - 是否涉及核心逻辑
   - 风险说明

### 第五步：等待审核

- PR 提交后会自动运行 GitHub Actions 检查
- 如果 Actions 失败，需要修复后重新提交
- 等待负责人审核，未经批准不合并到 main

## 禁止修改的范围

以下内容**禁止擅自修改**，如发现问题请记录到 Issue 中：

- `Control/` — 核心控制逻辑
- `Planner/` — 路径规划逻辑
- `SLAM/` — 算法逻辑
- `MissionControl/` — 任务控制逻辑
- PX4 / MAVROS 相关飞控配置和实机飞行参数
- 第三方依赖、许可证声明、版权归属
- 大规模删除文件、迁移大文件、修改历史记录
- 正式 Release 发布

## 允许修改的范围

- 文档中的明显错别字、脚本名称不一致、AI 残留
- 维护文件（CONTRIBUTING.md、SECURITY.md、PR 模板等）
- Python 明显语法错误、launch 明显路径错误、package.xml 模板残留
- GitHub Actions 静态检查配置

## AI 使用规范

### 允许使用 AI 的场景
- 理解代码、解释目录结构、分析报错日志
- 生成文档初稿、PR 描述草稿、Issue 草稿
- 生成 GitHub Actions 初稿
- 辅助查找低风险问题
- 辅助整理检查命令和报告格式

### 禁止或需确认的场景
- 让 AI 大规模重构项目
- 未理解就提交 AI 生成代码
- 让 AI 修改飞控、规划、控制、SLAM 核心逻辑
- 让 AI 编造测试结果或项目能力
- 让 AI 作为 commit author 或直接提交到 main

## 提问与反馈

如有疑问，请通过 Issue 或项目指定的联系方式进行沟通。
