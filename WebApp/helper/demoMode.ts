function isPlaceholderMongoUri(uri: string) {
  if (!uri) return true;
  return (
    uri.includes("YOUR_USER") ||
    uri.includes("YOUR_PASSWORD") ||
    uri.includes("cluster0.ev0ma.mongodb.net")
  );
}

export const isDemoMode = () => {
  if (process.env.DEMO_MODE === "true") return true;
  if (process.env.DEMO_MODE === "false") return false;
  return isPlaceholderMongoUri(process.env.MONGODB_URI || "");
};

export const DEMO_AUTH_TOKEN = "demo-mode";
export const DEMO_USERNAME = "RAG-ED Demo";
export const DEMO_USER_ID = "demo-user-id";
