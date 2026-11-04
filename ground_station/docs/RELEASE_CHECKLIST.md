# 比赛上传 / GitHub 发布清单

## 本次整理完成

- 独立源码快照，不携带旧 Git 历史和 KR260 飞控工程。
- 保留两个工程目录关系、PL/PS/驱动/服务/资源和软件测试。
- 清除调试备份、构建缓存、个人绝对路径和运行记录；不包含原始人像及账号认证脚本。
- 二进制部署资源与 GitHub 源码分包，提供 SHA-256 清单。
- 统一 README、架构、安装、测试、性能边界和第三方说明。

## 比赛提交时进一步确认

- 队伍名称、作者署名、代码与地图图片权利归属。
- 自研部分已由用户确认 MIT，署名 LT123456-abc；根 LICENSE 与 LICENSE.md 说明范围。
- 比赛要求的仓库地址、提交版本、第三方代码/AI 辅助使用披露格式及比赛截止日期。
- 若比赛限制预编译文件，提交源码包并按规则提供附件；不要把运行包直接当纯开源源码上传。
- 重新构建硬件、干净 SD 卡安装、真实板卡冷启动等未在本次整理中重测。

## 上传方法

本次按用户确认配置发布到 https://github.com/LT123456-abc/pynq-z2-ground-station。手动复现发布流程如下：

```bash
git init -b main
git add .
git commit -m "Initial ground station source release"
git remote add origin https://github.com/YOUR_ACCOUNT/YOUR_REPOSITORY.git
git push -u origin main
```

先使用 `git diff --cached --stat` 和本包清单检查提交内容。不要在原始包含 KR260 工程的工作目录直接 git add 全部文件。
