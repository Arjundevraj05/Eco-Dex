const { MongoMemoryServer } = require("mongodb-memory-server");
const { spawn } = require("child_process");
const path = require("path");

async function main() {
  const mongod = await MongoMemoryServer.create({
    instance: {
      dbName: "ecodex",
    },
    binary: {
      version: "6.0.16",
    },
  });
  const uri = mongod.getUri("ecodex");

  console.log(`Starting in-memory MongoDB at ${uri}`);

  const child = spawn("npx next dev", [], {
    cwd: path.join(__dirname, ".."),
    stdio: "inherit",
    shell: true,
      env: {
        ...process.env,
        MONGODB_URI: uri,
        MONGODB_DB: "ecodex",
        JWT_SECRET: process.env.JWT_SECRET || "local-dev-jwt-secret-change-me",
        TOKEN_SECRET: process.env.TOKEN_SECRET || "local-dev-token-secret",
        NEXT_PUBLIC_BASE_URL: "http://localhost:3000",
        DOMAIN: "http://localhost:3000",
        FLASK_BACKEND_URL: "http://localhost:5000",
        NEXT_PUBLIC_FLASK_URL: "http://localhost:5000",
      },
  });

  const shutdown = async () => {
    child.kill();
    await mongod.stop();
    process.exit(0);
  };

  process.on("SIGINT", shutdown);
  process.on("SIGTERM", shutdown);
  child.on("exit", async (code) => {
    await mongod.stop();
    process.exit(code ?? 0);
  });
}

main().catch((error) => {
  console.error("Failed to start local dev environment:", error);
  process.exit(1);
});
