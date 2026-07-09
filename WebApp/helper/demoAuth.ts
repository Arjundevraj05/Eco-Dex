import { NextResponse } from "next/server";
import { serialize } from "cookie";
import {
  DEMO_AUTH_TOKEN,
  DEMO_USERNAME,
  DEMO_USER_ID,
  isDemoMode,
} from "./demoMode";

export function getDemoAuth() {
  return { id: DEMO_USER_ID, username: DEMO_USERNAME };
}

export function withDemoCookies(response: NextResponse) {
  const cookieOptions = {
    httpOnly: true,
    maxAge: 60 * 60 * 24 * 30,
    path: "/",
  };

  response.headers.append(
    "Set-Cookie",
    serialize("auth_token", DEMO_AUTH_TOKEN, cookieOptions)
  );
  response.headers.append(
    "Set-Cookie",
    serialize("username", DEMO_USERNAME, cookieOptions)
  );

  return response;
}

export function isDemoToken(token?: string) {
  return isDemoMode() && token === DEMO_AUTH_TOKEN;
}
