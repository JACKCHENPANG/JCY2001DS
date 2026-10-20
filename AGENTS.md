# JCY2001D 多 AI 协作规则

此目录是 JCY2001D 固件的**发布集成目录**。多名 AI 不能同时直接修改这里的
`src/`、`include/`、`platformio*.ini` 或版本号；否则很容易互相覆盖代码、重复
版本号，或烧录到错误的二进制。

## 统一流程

### 1. 功能开发 AI

1. 先执行 `./scripts/jcy_agent_guard.sh status`。
2. 用独立工作树开发：
   ```bash
   ./scripts/jcy_agent_worktree.sh create <agent>-<short-task>
   ```
3. 只在创建出的工作树和 `codex/<agent>-<short-task>` 分支修改、编译、提交。
4. 不在功能分支修改 `JCY1000_FIRMWARE_VERSION`；完成后交付提交哈希给集成 AI。

### 2. 发布集成 AI（每次只能一个）

1. 在本目录获取锁：
   ```bash
   ./scripts/jcy_agent_guard.sh acquire <agent> "<task>"
   ```
2. 合并已验证的功能提交，检查差异、编译和截图。
3. 仅由集成 AI 递增版本：
   ```bash
   ./scripts/jcy_release_version.py bump \
     --owner <agent> --note "short release note"
   ```
4. 单独提交版本、构建结果说明并推送；**只有用户明确授权后才可烧录**。
5. 完成后释放锁：
   ```bash
   ./scripts/jcy_agent_guard.sh release <agent>
   ```

## 强制约定

- 锁被占用或发布目录有未提交源码改动时，停止操作，不要覆盖、`reset`、`stash` 或删除他人的文件。
- `.jcy-agent-lock/` 是本机互斥锁，不能手动删除；不知道持有人时先询问用户。
- `backups/`、`diagnostics/`、`screenshots/` 不可混入源码提交。
- 本仓库已启用 `.githooks/pre-commit`：提交 `src/main.cpp` 时，构建标签必须包含当前固件版本号。
- 烧录、输出开启、恢复出厂、云端发布仍分别需要用户明确授权。
