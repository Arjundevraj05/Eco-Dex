import { NextResponse } from "next/server";
import jwt from "jsonwebtoken";
import { getDemoAuth, isDemoToken } from "@/helper/demoAuth";
import { isDemoMode } from "@/helper/demoMode";

const JWT_SECRET = process.env.JWT_SECRET as string;

export async function GET(request: Request) {
  const authCookie = request.headers
    .get("cookie")
    ?.split("; ")
    .find((c) => c.startsWith("auth_token="));

  if (!authCookie) {
    if (isDemoMode()) {
      return NextResponse.json({ username: getDemoAuth().username });
    }
    return NextResponse.json({ error: "Not authenticated" }, { status: 401 });
  }

  const token = authCookie.split("=")[1];

  if (isDemoToken(token)) {
    return NextResponse.json({ username: getDemoAuth().username });
  }

  try {
    const decoded = jwt.verify(token, JWT_SECRET) as { username: string };
    return NextResponse.json({ username: decoded.username });
  } catch {
    return NextResponse.json({ error: "Invalid token" }, { status: 403 });
  }
}
