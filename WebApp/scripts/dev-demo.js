const { spawn } = require("child_process");
const path = require("path");

process.env.DEMO_MODE = "true";

const child = spawn(
  process.platform === "win32" ? "npx.cmd" : "npx",
  ["next", "dev"],
  {
    cwd: path.join(__dirname, ".."),
    stdio: "inherit",
    shell: true,
    env: { ...process.env, DEMO_MODE: "true" },
  }
);

child.on("exit", (code) => process.exit(code ?? 0));
