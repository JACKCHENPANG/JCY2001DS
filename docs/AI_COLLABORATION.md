# JCY2001D 多 AI 协作说明

你不需要手动分支、改版本号或处理冲突；参与此固件的 AI 必须遵循根目录
`AGENTS.md`。

## 为什么要这样做

过去所有 AI 都直接写同一目录时，可能发生：

- 两个 AI 都使用 `A70`；
- 后写入的人覆盖先写入的人；
- 编译的是一个 AI 的代码，烧录的是另一个 AI 的二进制；
- 未提交的修复被误清理。

现在分成两类角色：

1. **功能 AI**：自动在独立 Git worktree / 分支开发，只交付一个提交。
2. **发布集成 AI**：一次只能一个，持有本机锁，合并功能、递增版本、编译、
   推送，并在你明确确认后烧录。

## AI 实际执行的命令

```bash
# 查看当前是否有人集成版本
./scripts/jcy_agent_guard.sh status

# 功能开发：创建隔离目录，不碰发布目录
./scripts/jcy_agent_worktree.sh create codex-header-layout

# 发布集成：取得独占锁后，自动从 A69 变为 A70
./scripts/jcy_agent_guard.sh acquire codex-release "merge header layout"
./scripts/jcy_release_version.py bump --owner codex-release --note "header spacing"
pio run -e JC8048W550_VENDOR8
./scripts/jcy_agent_guard.sh release codex-release
```

锁被占用或发布目录有未提交源码改动时，AI 必须停止并报告，不能覆盖对方工作。
烧录、打开输出、发布云端仍需单独得到你的确认。
