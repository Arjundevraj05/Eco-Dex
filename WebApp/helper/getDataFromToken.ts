import { NextRequest } from "next/server";
import { getAuthFromRequestOrDemo } from "./getAuthFromRequest";

export const getDataFromToken = (request: NextRequest) => {
  const auth = getAuthFromRequestOrDemo(request);
  if (!auth) {
    throw new Error("Not authenticated");
  }
  return auth.id;
};
