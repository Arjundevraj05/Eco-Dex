import { NextRequest } from "next/server";
import jwt from "jsonwebtoken";
import { getDemoAuth, isDemoToken } from "./demoAuth";
import { isDemoMode } from "./demoMode";

const JWT_SECRET = process.env.JWT_SECRET as string;

export interface AuthPayload {
  id: string;
  username: string;
}

export function getAuthFromRequest(request: NextRequest): AuthPayload | null {
  const token = request.cookies.get("auth_token")?.value;

  if (isDemoToken(token)) {
    return getDemoAuth();
  }

  if (!token || !JWT_SECRET) return null;

  try {
    return jwt.verify(token, JWT_SECRET) as AuthPayload;
  } catch {
    return null;
  }
}

export function getAuthFromRequestOrDemo(request: NextRequest): AuthPayload | null {
  if (isDemoMode()) {
    return getDemoAuth();
  }

  return getAuthFromRequest(request);
}
