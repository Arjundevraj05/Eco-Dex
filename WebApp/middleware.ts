import { NextResponse } from "next/server";
import type { NextRequest } from "next/server";
import { withDemoCookies } from "@/helper/demoAuth";
import { DEMO_AUTH_TOKEN, isDemoMode } from "@/helper/demoMode";

export function middleware(request: NextRequest) {
  const authToken = request.cookies.get("auth_token")?.value;
  const isAuthenticated =
    !!authToken && (!isDemoMode() || authToken === DEMO_AUTH_TOKEN);

  const protectedRoutes = ["/", "/livecam", "/map", "/override", "/reports"];
  const authRoutes = ["/signin", "/signup"];
  const { pathname } = request.nextUrl;

  if (isDemoMode()) {
    if (authRoutes.includes(pathname)) {
      return NextResponse.redirect(new URL("/", request.url));
    }

    if (!isAuthenticated && protectedRoutes.includes(pathname)) {
      return withDemoCookies(NextResponse.next());
    }

    return NextResponse.next();
  }

  if (!isAuthenticated && protectedRoutes.includes(pathname)) {
    return NextResponse.redirect(new URL("/signin", request.url));
  }

  if (isAuthenticated && authRoutes.includes(pathname)) {
    return NextResponse.redirect(new URL("/", request.url));
  }

  return NextResponse.next();
}

export const config = {
  matcher: ["/", "/livecam", "/map", "/override", "/reports", "/signin", "/signup"],
};
