# 基础镜像按补丁版本 + digest 双重锁定；换镜像时用 docker buildx imagetools inspect 取多架构 index digest。
FROM python:3.12.7-slim-bookworm@sha256:60d9996b6a8a3689d36db740b49f4327be3be09a21122bd02fb8895abb38b50d

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

# 解题与参考解的身份由 task.toml 的 [agent] user = "runner" 指定。
# 不写 USER：Harbor 的保活进程随默认用户运行，评分前清理 runner 进程时不能把它一起结束。
RUN useradd --create-home --uid 1000 runner

# Agent runtime tooling is preinstalled so model runs do not repeat the slow network setup.
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates curl nodejs npm procps \
    && npm install -g @anthropic-ai/claude-code \
    && npm cache clean --force \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY app/ /app/
RUN chown -R runner:runner /app
