import dotenv from "dotenv";
dotenv.config({ path: ".env.local" });
dotenv.config({ path: ".env" });

function isPlaceholderMongoUri(uri) {
  if (!uri) return true;
  return (
    uri.includes("YOUR_USER") ||
    uri.includes("YOUR_PASSWORD") ||
    uri.includes("cluster0.ev0ma.mongodb.net")
  );
}

function shouldUseDemoMode() {
  if (process.env.DEMO_MODE === "true") return true;
  if (process.env.DEMO_MODE === "false") return false;
  return isPlaceholderMongoUri(process.env.MONGODB_URI || "");
}

const demoEnabled = shouldUseDemoMode();

/** @type {import('next').NextConfig} */
const nextConfig = {
  env: {
    DEMO_MODE: demoEnabled ? "true" : "false",
    NEXT_PUBLIC_DEMO_MODE: demoEnabled ? "true" : "false",
  },
  eslint: {
    ignoreDuringBuilds: true,
  },
  images: {
    remotePatterns: [{ protocol: "https", hostname: "ui-avatars.com" }],
  },
};

export default nextConfig;
